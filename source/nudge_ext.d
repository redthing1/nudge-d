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

    public enum float[3] zero_vec = [0, 0, 0];
    public enum nudge.Transform identity_transform = nudge.Transform([0, 0, 0],
                0, [0.0f, 0.0f, 0.0f, 1.0f]);
    public enum nudge.BodyMomentum zero_momentum = nudge.BodyMomentum(zero_vec, 0, zero_vec, 0);

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
        public uint append_body(nudge.Transform transform,
                nudge.BodyProperties properties, nudge.BodyMomentum momentum) {
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
        public uint append_box_collider(uint body_id, nudge.BoxCollider collider,
                nudge.Transform transform, uint tag = 0) {
            if (colliders.boxes.count >= max_boxes) {
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

        /// append a sphere collider and get its index
        public uint append_sphere_collider(uint body_id,
                nudge.SphereCollider collider, nudge.Transform transform, uint tag = 0) {
            if (colliders.spheres.count >= max_spheres) {
                assert(0, "max body count exceeded (spheres)");
            }

            uint sphere_id = colliders.spheres.count++;

            colliders.spheres.tags[sphere_id] = tag;
            colliders.spheres.data[sphere_id] = collider;
            colliders.spheres.transforms[sphere_id] = transform;
            colliders.spheres.transforms[sphere_id].body = body_id;

            return sphere_id;
        }

        /// zero the data of a sphere collider index
        public void clear_sphere_collider(uint id) {
            colliders.spheres.tags[id] = 0;
            memset(&colliders.spheres.data[id], 0, colliders.spheres.data[id].sizeof);
            memset(&colliders.spheres.transforms[id], 0, colliders.spheres.transforms[id].sizeof);
        }

        /// swap two sphere colliders given their indices
        public void swap_sphere_colliders(uint id_src, uint id_dst) {
            auto tmp_transform = colliders.spheres.transforms[id_dst];
            colliders.spheres.transforms[id_dst] = colliders.spheres.transforms[id_src];
            colliders.spheres.transforms[id_src] = tmp_transform;

            auto tmp_data = colliders.spheres.data[id_dst];
            colliders.spheres.data[id_dst] = colliders.spheres.data[id_src];
            colliders.spheres.data[id_src] = tmp_data;

            auto tmp_tag = colliders.spheres.tags[id_dst];
            colliders.spheres.tags[id_dst] = colliders.spheres.tags[id_src];
            colliders.spheres.tags[id_src] = tmp_tag;
        }

        /// pop the last sphere collider off
        public void pop_last_sphere_collider() {
            colliders.spheres.count--;
        }
    }
}

unittest {
    // try setting one up

    auto realm = new NudgeRealm(64, 64, 64);

    realm.allocate();
    realm.destroy();
}
