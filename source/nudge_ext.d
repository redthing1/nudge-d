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
        /// create a box body, and return the id
        public uint add_box(float mass, float collider_x, float collider_y, float collider_z) {
            if (bodies.count == max_bodies || colliders.boxes.count == max_boxes)
                return 0;

            uint new_body = bodies.count++;
            uint collider = colliders.boxes.count++;

            float k = mass * (1.0f / 3.0f);

            float kcx2 = k * collider_x * collider_x;
            float kcy2 = k * collider_y * collider_y;
            float kcz2 = k * collider_z * collider_z;

            nudge.BodyProperties properties = {};
            properties.mass_inverse = 1.0f / mass;
            properties.inertia_inverse[0] = 1.0f / (kcy2 + kcz2);
            properties.inertia_inverse[1] = 1.0f / (kcx2 + kcz2);
            properties.inertia_inverse[2] = 1.0f / (kcx2 + kcy2);

            memset(&bodies.momentum[new_body], 0, bodies.momentum[new_body].sizeof);
            bodies.idle_counters[new_body] = 0;
            bodies.properties[new_body] = properties;
            bodies.transforms[new_body] = identity_transform;

            colliders.boxes.transforms[collider] = identity_transform;
            colliders.boxes.transforms[collider].body = new_body;

            colliders.boxes.data[collider].size[0] = collider_x;
            colliders.boxes.data[collider].size[1] = collider_y;
            colliders.boxes.data[collider].size[2] = collider_z;
            colliders.boxes.tags[collider] = collider;

            return new_body;
        }

        /// create a sphere body, and return the id
        public uint add_sphere(float mass, float radius) {
            if (bodies.count == max_bodies || colliders.spheres.count == max_spheres)
                return 0;

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
