module nudge;

extern (C++,nudge) {
    struct Arena {
        void* data;
        size_t size;
    }

    struct Transform {
        float[3] position;
        uint body;
        float[4] rotation;
    }

    struct BodyProperties {
        float[3] inertia_inverse;
        float mass_inverse;
    }

    struct BodyMomentum {
        float[3] velocity;
        float unused0;
        float[3] angular_velocity;
        float unused1;
    }

    struct SphereCollider {
        float radius;
    }

    struct BoxCollider {
        float[3] size;
        float unused;
    }

    struct Contact {
        float[3] position;
        float penetration;
        float[3] normal;
        float friction;
    }

    struct BodyPair {
        ushort a;
        ushort b;
    }

    struct ContactData {
        Contact* data;
        BodyPair* bodies;
        ulong* tags;
        uint capacity;
        uint count;

        uint* sleeping_pairs;
        uint sleeping_count;
    }

    struct ColliderDataBoxes {
        uint* tags;
        BoxCollider* data;
        Transform* transforms;
        uint count;
    }

    struct ColliderDataSpheres {
        uint* tags;
        SphereCollider* data;
        Transform* transforms;
        uint count;
    }

    struct ColliderData {
        ColliderDataBoxes boxes;
        ColliderDataSpheres spheres;
    }

    struct BodyData {
        Transform* transforms;
        BodyProperties* properties;
        BodyMomentum* momentum;
        ubyte* idle_counters;
        uint count;
    }

    struct BodyConnections {
        BodyPair* data;
        uint count;
    }

    struct CachedContactImpulse {
        float[3] impulse;
        float unused;
    }

    struct ContactCache {
        ulong* tags;
        CachedContactImpulse* data;
        uint capacity;
        uint count;
    }

    struct ActiveBodies {
        ushort* indices;
        uint capacity;
        uint count;
    }

    struct ContactImpulseData;
    struct ContactConstraintData;

    void collide(ActiveBodies* active_bodies, ContactData* contacts, BodyData bodies,
            ColliderData colliders, BodyConnections body_connections, Arena temporary);

    ContactImpulseData* read_cached_impulses(ContactCache contact_cache,
            ContactData contacts, Arena* memory);

    void write_cached_impulses(ContactCache* contact_cache,
            ContactData contacts, ContactImpulseData* contact_impulses);

    ContactConstraintData* setup_contact_constraints(ActiveBodies active_bodies,
            ContactData contacts, BodyData bodies,
            ContactImpulseData* contact_impulses, Arena* memory);

    void apply_impulses(ContactConstraintData* data, BodyData bodies);

    void update_cached_impulses(ContactConstraintData* data, ContactImpulseData* contact_impulses);

    void advance(ActiveBodies active_bodies, BodyData bodies, float time_step);
}
