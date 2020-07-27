module nudge_ext;

import core.memory;
import core.stdc.string : memset;
static import nudge;

/// represents a context for nudge
class NudgeRealm {
    public const(uint) max_bodies;
    public const(uint) max_boxes;
    public const(uint) max_spheres;

    public size_t arena_size = 64 * 1024 * 1024; // 64M

    public enum nudge.Transform identity_transform = nudge.Transform([0, 0, 0],
                0, [0.0f, 0.0f, 0.0f, 1.0f]);

    public nudge.Arena arena;
    public nudge.BodyData bodies;
    public nudge.ColliderData colliders;
    public nudge.ContactData contact_data;
    public nudge.ContactCache contact_cache;
    public nudge.ActiveBodies active_bodies;

    /// initializes a nudge realm with the given maximums
    this(uint max_bodies, uint max_boxes, uint max_spheres) {
        assert(max_boxes <= max_bodies, "boxes exceeds max body count");
        assert(max_spheres <= max_bodies, "spheres exceeds max body count");

        this.max_bodies = max_bodies;
        this.max_boxes = max_boxes;
        this.max_spheres = max_spheres;
    }

    /// allocate memory for nudge
    public void allocate() {
        // Allocate memory for simulation arena.
        arena.size = arena_size;
        arena.data = GC.malloc(arena.size);

        // Allocate memory for bodies, colliders, and contacts.
        active_bodies.capacity = max_bodies;
        active_bodies.indices = new ushort[max_bodies].ptr;

        bodies.idle_counters = new ubyte[max_bodies].ptr;
        bodies.transforms = new nudge.Transform[max_bodies].ptr;
        bodies.momentum = new nudge.BodyMomentum[max_bodies].ptr;
        bodies.properties = new nudge.BodyProperties[max_bodies].ptr;

        colliders.boxes.data = new nudge.BoxCollider[max_boxes].ptr;
        colliders.boxes.tags = new uint[max_boxes].ptr;
        colliders.boxes.transforms = new nudge.Transform[max_boxes].ptr;

        colliders.spheres.data = new nudge.SphereCollider[max_spheres].ptr;
        colliders.spheres.tags = new uint[max_spheres].ptr;
        colliders.spheres.transforms = new nudge.Transform[max_spheres].ptr;

        contact_data.capacity = max_bodies * 64;
        contact_data.bodies = new nudge.BodyPair[contact_data.capacity].ptr;
        contact_data.data = new nudge.Contact[contact_data.capacity].ptr;
        contact_data.tags = new ulong[contact_data.capacity].ptr;
        contact_data.sleeping_pairs = new uint[contact_data.capacity].ptr;

        contact_cache.capacity = max_bodies * 64;
        contact_cache.data = new nudge.CachedContactImpulse[contact_cache.capacity].ptr;
        contact_cache.tags = new ulong[contact_cache.capacity].ptr;
    }

    public void destroy() {
        // free the arena data, because we used malloc()
        GC.free(arena.data);
    }

    pragma(inline) {
        /// append a new body, return the id
        public uint append_body(ref nudge.Transform transform,
                ref nudge.BodyProperties properties, ref nudge.BodyMomentum momentum) {
            if (bodies.count >= max_bodies) {
                assert(0, "max body count exceeded");
            }

            uint id = bodies.count++;

            bodies.transforms[id] = transform;
            bodies.properties[id] = properties;
            bodies.momentum[id] = momentum;
            bodies.idle_counters[id] = 0;

            return id;
        }

        /// pop the last body off
        public void pop_last_body() {
            bodies.count--;
        }

        /// clear the data of a body with matching id
        public void clear_body(uint id) {
            // zero out the body
            bodies.transforms[id] = identity_transform;
            memset(&bodies.properties[id], 0, bodies.properties[id].sizeof);
            memset(&bodies.momentum[id], 0, bodies.momentum[id].sizeof);
            bodies.idle_counters[id] = 0;
        }

        /// swap two bodies given their indices
        public void swap_bodies(uint id_src, uint id_dst) {
            auto tmp_transform = bodies.transforms[id_dst];
            bodies.transforms[id_dst] = bodies.transforms[id_src];
            bodies.transforms[id_src] = tmp_transform;

            auto tmp_props = bodies.properties[id_dst];
            bodies.properties[id_dst] = bodies.properties[id_src];
            bodies.properties[id_src] = tmp_props;

            auto tmp_momentum = bodies.momentum[id_dst];
            bodies.momentum[id_dst] = bodies.momentum[id_src];
            bodies.momentum[id_src] = tmp_momentum;

            auto tmp_idle = bodies.idle_counters[id_dst];
            bodies.idle_counters[id_dst] = bodies.idle_counters[id_src];
            bodies.idle_counters[id_src] = tmp_idle;
        }

        /// append a box collider and get its index
        public uint append_box_collider(uint body_id,
                ref nudge.BoxCollider collider, ref nudge.Transform transform, uint tag) {
            if (colliders.boxes.count >= max_bodies) {
                assert(0, "max body count exceeded (boxes)");
            }

            uint box_id = colliders.boxes.count++;

            colliders.boxes.tags[box_id] = tag;
            colliders.boxes.data[box_id] = collider;
            colliders.boxes.transforms[box_id] = transform;
            colliders.boxes.transforms[box_id].body = body_id;

            return box_id;
        }

        /// zero the data of a box collider index
        public void clear_box_collider(uint id) {
            colliders.boxes.tags[id] = 0;
            memset(&colliders.boxes.data[id], 0, colliders.boxes.data[id].sizeof);
            memset(&colliders.boxes.transforms[id], 0, colliders.boxes.transforms[id].sizeof);
        }

        /// swap two box colliders given their indices
        public void swap_box_colliders(uint id_src, uint id_dst) {
            auto tmp_transform = colliders.boxes.transforms[id_dst];
            colliders.boxes.transforms[id_dst] = colliders.boxes.transforms[id_src];
            colliders.boxes.transforms[id_src] = tmp_transform;

            auto tmp_data = colliders.boxes.data[id_dst];
            colliders.boxes.data[id_dst] = colliders.boxes.data[id_src];
            colliders.boxes.data[id_src] = tmp_data;

            auto tmp_tag = colliders.boxes.tags[id_dst];
            colliders.boxes.tags[id_dst] = colliders.boxes.tags[id_src];
            colliders.boxes.tags[id_src] = tmp_tag;
        }

        /// pop the last box collider off
        public void pop_last_box_collider() {
            colliders.boxes.count--;
        }

        /// create a box collider, and return the id
        public uint add_box(float mass, float collider_x, float collider_y, float collider_z) {
            if (bodies.count == max_bodies || colliders.boxes.count == max_boxes) {
                assert(0, "max body count exceeded (boxes)");
            }

            uint new_body = bodies.count++;
            uint collider = colliders.boxes.count++;

            // calculate some Very Cool physics stuff
            float k = mass * (1.0f / 3.0f);
            float kcx2 = k * collider_x * collider_x;
            float kcy2 = k * collider_y * collider_y;
            float kcz2 = k * collider_z * collider_z;

            // set body values
            nudge.BodyProperties properties = {};
            properties.mass_inverse = 1.0f / mass;
            properties.inertia_inverse[0] = 1.0f / (kcy2 + kcz2);
            properties.inertia_inverse[1] = 1.0f / (kcx2 + kcz2);
            properties.inertia_inverse[2] = 1.0f / (kcx2 + kcy2);
            memset(&bodies.momentum[new_body], 0, bodies.momentum[new_body].sizeof);
            bodies.idle_counters[new_body] = 0;
            bodies.properties[new_body] = properties;
            bodies.transforms[new_body] = identity_transform;

            // set collider position
            colliders.boxes.transforms[collider] = identity_transform;
            colliders.boxes.transforms[collider].body = new_body;

            // set collider size
            colliders.boxes.data[collider].size[0] = collider_x;
            colliders.boxes.data[collider].size[1] = collider_y;
            colliders.boxes.data[collider].size[2] = collider_z;
            colliders.boxes.tags[collider] = collider;

            return new_body;
        }

        // public void remove_box(uint id) {
        //     // look up collider id
        //     auto coll_id = 0; // magic

        //     // clear collider size
        //     colliders.boxes.data[coll_id].size[0] = 0;
        //     colliders.boxes.data[coll_id].size[1] = 0;
        //     colliders.boxes.data[coll_id].size[2] = 0;

        //     // clear collider position
        //     colliders.boxes.transforms[coll_id] = identity_transform;
        //     colliders.boxes.transforms[coll_id].body = 0;

        //     // clear body values
        //     // ?

        //     colliders.boxes.count--;
        //     bodies.count--;
        // }

        /// create a sphere collider, and return the id
        public uint add_sphere(float mass, float radius) {
            if (bodies.count == max_bodies || colliders.spheres.count == max_spheres) {
                assert(0, "max body count exceeded (spheres)");
            }

            uint new_body = bodies.count++;
            uint collider = colliders.spheres.count++;

            float k = 2.5f / (mass * radius * radius);

            nudge.BodyProperties properties = {};
            properties.mass_inverse = 1.0f / mass;
            properties.inertia_inverse[0] = k;
            properties.inertia_inverse[1] = k;
            properties.inertia_inverse[2] = k;

            memset(&bodies.momentum[new_body], 0, bodies.momentum[new_body].sizeof);
            bodies.idle_counters[new_body] = 0;
            bodies.properties[new_body] = properties;
            bodies.transforms[new_body] = identity_transform;

            colliders.spheres.transforms[collider] = identity_transform;
            colliders.spheres.transforms[collider].body = new_body;

            colliders.spheres.data[collider].radius = radius;
            colliders.spheres.tags[collider] = collider + max_boxes;

            return new_body;
        }
    }
}
