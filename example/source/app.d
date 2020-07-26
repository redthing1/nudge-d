import std.stdio;
import core.thread;

// import core.memory;
import core.stdc.string : memcpy, memset;
import core.stdc.stdlib : malloc, rand, RAND_MAX;

static import nudge;

static const uint max_body_count = 2048;
static const uint max_box_count = 2048;
static const uint max_sphere_count = 2048;

static const nudge.Transform identity_transform = nudge.Transform([0, 0, 0], 0,
		[0.0f, 0.0f, 0.0f, 1.0f]);

static nudge.Arena arena;
static nudge.BodyData bodies;
static nudge.ColliderData colliders;
static nudge.ContactData contact_data;
static nudge.ContactCache contact_cache;
static nudge.ActiveBodies active_bodies;

pragma(inline) {
	static void quaternion_concat(float[4] r, const float[4] a, const float[4] b) {
		r[0] = b[0] * a[3] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
		r[1] = b[1] * a[3] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
		r[2] = b[2] * a[3] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
		r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
	}

	static void quaternion_transform(float[3] r, const float[4] a, const float[3] b) {
		float[3] t;
		t[0] = a[1] * b[2] - a[2] * b[1];
		t[1] = a[2] * b[0] - a[0] * b[2];
		t[2] = a[0] * b[1] - a[1] * b[0];

		t[0] += t[0];
		t[1] += t[1];
		t[2] += t[2];

		r[0] = b[0] + a[3] * t[0] + a[1] * t[2] - a[2] * t[1];
		r[1] = b[1] + a[3] * t[1] + a[2] * t[0] - a[0] * t[2];
		r[2] = b[2] + a[3] * t[2] + a[0] * t[1] - a[1] * t[0];
	}

	static uint add_box(float mass, float cx, float cy, float cz) {
		if (bodies.count == max_body_count || colliders.boxes.count == max_box_count)
			return 0;

		uint new_body = bodies.count++;
		uint collider = colliders.boxes.count++;

		float k = mass * (1.0f / 3.0f);

		float kcx2 = k * cx * cx;
		float kcy2 = k * cy * cy;
		float kcz2 = k * cz * cz;

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

		colliders.boxes.data[collider].size[0] = cx;
		colliders.boxes.data[collider].size[1] = cy;
		colliders.boxes.data[collider].size[2] = cz;
		colliders.boxes.tags[collider] = cast(ushort) collider;

		return new_body;
	}

	static uint add_sphere(float mass, float radius) {
		if (bodies.count == max_body_count || colliders.spheres.count == max_sphere_count)
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
		colliders.spheres.tags[collider] = cast(ushort) (collider + max_box_count);

		return new_body;
	}
}

void simulate() {
	static const uint steps = 2;
	static const uint iterations = 20;

	float time_step = 1.0f / (60.0f * cast(float) steps);

	for (uint n = 0; n < steps; ++n) {
		// Setup a temporary memory arena. The same temporary memory is reused each iteration.
		nudge.Arena temporary = arena;

		// Find contacts.
		nudge.BodyConnections connections = {}; // NOTE: Custom constraints should be added as body connections.
		nudge.collide(&active_bodies, &contact_data, bodies, colliders, connections, temporary);

		// NOTE: Custom contacts can be added here, e.g., against the static environment.

		// Apply gravity and damping.
		float damping = 1.0f - time_step * 0.25f;

		for (uint i = 0; i < active_bodies.count; ++i) {
			uint index = active_bodies.indices[i];

			bodies.momentum[index].velocity[1] -= 9.82f * time_step;

			bodies.momentum[index].velocity[0] *= damping;
			bodies.momentum[index].velocity[1] *= damping;
			bodies.momentum[index].velocity[2] *= damping;

			bodies.momentum[index].angular_velocity[0] *= damping;
			bodies.momentum[index].angular_velocity[1] *= damping;
			bodies.momentum[index].angular_velocity[2] *= damping;
		}

		// Read previous impulses from contact cache.
		nudge.ContactImpulseData* contact_impulses = nudge.read_cached_impulses(contact_cache,
				contact_data, &temporary);

		// Setup contact constraints and apply the initial impulses.
		nudge.ContactConstraintData* contact_constraints = nudge.setup_contact_constraints(
				active_bodies, contact_data, bodies, contact_impulses, &temporary);

		// Apply contact impulses. Increasing the number of iterations will improve stability.
		for (uint i = 0; i < iterations; ++i) {
			nudge.apply_impulses(contact_constraints, bodies);
			// NOTE: Custom constraint impulses should be applied here.
		}

		// Update contact impulses.
		nudge.update_cached_impulses(contact_constraints, contact_impulses);

		// Write the updated contact impulses to the cache.
		nudge.write_cached_impulses(&contact_cache, contact_data, contact_impulses);

		// Move active bodies.
		nudge.advance(active_bodies, bodies, time_step);
	}
}

void dump() {
	// Render boxes.
	for (uint i = 0; i < colliders.boxes.count; ++i) {
		uint body = colliders.boxes.transforms[i].body;

		float[3] scale;
		float[4] rotation;
		float[3] position;

		memcpy(cast(void*) scale, cast(void*) colliders.boxes.data[i].size, scale.sizeof);

		quaternion_concat(rotation, bodies.transforms[body].rotation,
				colliders.boxes.transforms[i].rotation);
		quaternion_transform(position, bodies.transforms[body].rotation,
				colliders.boxes.transforms[i].position);

		position[0] += bodies.transforms[body].position[0];
		position[1] += bodies.transforms[body].position[1];
		position[2] += bodies.transforms[body].position[2];

		writefln("cube: pos(%s), rot(%s), scale(%s)", position, rotation, scale);
	}

	// Render spheres.
	for (uint i = 0; i < colliders.spheres.count; ++i) {
		uint body = colliders.spheres.transforms[i].body;

		float[3] scale;
		float[4] rotation;
		float[3] position;

		scale[0] = scale[1] = scale[2] = colliders.spheres.data[i].radius;

		quaternion_concat(rotation, bodies.transforms[body].rotation,
				colliders.spheres.transforms[i].rotation);
		quaternion_transform(position, bodies.transforms[body].rotation,
				colliders.spheres.transforms[i].position);

		position[0] += bodies.transforms[body].position[0];
		position[1] += bodies.transforms[body].position[1];
		position[2] += bodies.transforms[body].position[2];

		writefln("sphere: pos(%s), rot(%s), scale(%s)", position, rotation, scale);
	}
}

void main() {
	// setup

	// Allocate memory for simulation arena.
	arena.size = 64 * 1024 * 1024;
	arena.data = malloc(arena.size);

	// Allocate memory for bodies, colliders, and contacts.
	active_bodies.capacity = max_box_count;
	active_bodies.indices = cast(ushort*)(malloc(ushort.sizeof * max_body_count));

	bodies.idle_counters = cast(ubyte*)(malloc(ubyte.sizeof * max_body_count));
	bodies.transforms = cast(nudge.Transform*)(malloc(nudge.Transform.sizeof * max_body_count));
	bodies.momentum = cast(nudge.BodyMomentum*)(malloc(nudge.BodyMomentum.sizeof * max_body_count));
	bodies.properties = cast(nudge.BodyProperties*)(
			malloc(nudge.BodyProperties.sizeof * max_body_count));

	colliders.boxes.data = cast(nudge.BoxCollider*)(
			malloc(nudge.BoxCollider.sizeof * max_box_count));
	colliders.boxes.tags = cast(ushort*)(malloc(ushort.sizeof * max_box_count));
	colliders.boxes.transforms = cast(nudge.Transform*)(
			malloc(nudge.Transform.sizeof * max_box_count));

	colliders.spheres.data = cast(nudge.SphereCollider*)(
			malloc(nudge.SphereCollider.sizeof * max_sphere_count));
	colliders.spheres.tags = cast(ushort*)(malloc(ushort.sizeof * max_sphere_count));
	colliders.spheres.transforms = cast(nudge.Transform*)(
			malloc(nudge.Transform.sizeof * max_sphere_count));

	contact_data.capacity = max_body_count * 64;
	contact_data.bodies = cast(nudge.BodyPair*)(
			malloc(nudge.BodyPair.sizeof * contact_data.capacity));
	contact_data.data = cast(nudge.Contact*)(malloc(nudge.Contact.sizeof * contact_data.capacity));
	contact_data.tags = cast(ulong*)(malloc(ulong.sizeof * contact_data.capacity));
	contact_data.sleeping_pairs = cast(uint*)(malloc(uint.sizeof * contact_data.capacity));

	contact_cache.capacity = max_body_count * 64;
	contact_cache.data = cast(nudge.CachedContactImpulse*)(
			malloc(nudge.CachedContactImpulse.sizeof * contact_cache.capacity));
	contact_cache.tags = cast(ulong*)(malloc(ulong.sizeof * contact_cache.capacity));

	// The first body is the static world.
	bodies.count = 1;
	bodies.idle_counters[0] = 0;
	bodies.transforms[0] = identity_transform;

	// dlang already zeroes memory for us
	// memset(bodies.momentum, 0, sizeof(bodies.momentum[0]));
	// memset(bodies.properties, 0, sizeof(bodies.properties[0]));

	// Add ground.
	{
		uint collider = colliders.boxes.count++;

		colliders.boxes.transforms[collider] = identity_transform;
		colliders.boxes.transforms[collider].position[1] -= 20.0f;

		colliders.boxes.data[collider].size[0] = 400.0f;
		colliders.boxes.data[collider].size[1] = 10.0f;
		colliders.boxes.data[collider].size[2] = 400.0f;
		colliders.boxes.tags[collider] = cast(ushort) collider;
	}

	// Add boxes.
	for (uint i = 0; i < 1024; ++i) {
		float sx = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;
		float sy = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;
		float sz = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;

		uint body = add_box(8.0f * sx * sy * sz, sx, sy, sz);

		bodies.transforms[body].position[0] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;
		bodies.transforms[body].position[1] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 300.0f;
		bodies.transforms[body].position[2] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;
	}

	// Add spheres.
	for (uint i = 0; i < 512; ++i) {
		float s = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;

		uint body = add_sphere(4.18879f * s * s * s, s);

		bodies.transforms[body].position[0] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;
		bodies.transforms[body].position[1] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 300.0f;
		bodies.transforms[body].position[2] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;
	}

	while (true) {
		simulate();
		dump();
		Thread.sleep(dur!"msecs"(16));
	}
}
