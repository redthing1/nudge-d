module nudge_ext;

import core.memory;
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
}
