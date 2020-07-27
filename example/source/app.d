import std.stdio;
import core.thread;

import core.stdc.string : memcpy, memset;
import core.stdc.stdlib : malloc, rand, RAND_MAX;

static import nudge;
import nudge_ext;

static NudgeRealm realm;

pragma(inline) {
	static void quaternion_concat(ref float[4] r, const float[4] a, const float[4] b) {
		r[0] = b[0] * a[3] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
		r[1] = b[1] * a[3] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
		r[2] = b[2] * a[3] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
		r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
	}

	static void quaternion_transform(ref float[3] r, const float[4] a, const float[3] b) {
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
}

void simulate() {
	static const uint steps = 2;
	static const uint iterations = 20;

	float time_step = 1.0f / (60.0f * cast(float) steps);

	for (uint n = 0; n < steps; ++n) {
		// Setup a temporary memory arena. The same temporary memory is reused each iteration.
		nudge.Arena temporary = realm.arena;

		// Find contacts.
		nudge.BodyConnections connections = {}; // NOTE: Custom constraints should be added as body connections.
		nudge.collide(&realm.active_bodies, &realm.contact_data,
				realm.bodies, realm.colliders, connections, temporary);

		// NOTE: Custom contacts can be added here, e.g., against the static environment.

		// Apply gravity and damping.
		float damping = 1.0f - time_step * 0.25f;

		for (uint i = 0; i < realm.active_bodies.count; ++i) {
			uint index = realm.active_bodies.indices[i];

			realm.bodies.momentum[index].velocity[1] -= 9.82f * time_step;

			realm.bodies.momentum[index].velocity[0] *= damping;
			realm.bodies.momentum[index].velocity[1] *= damping;
			realm.bodies.momentum[index].velocity[2] *= damping;

			realm.bodies.momentum[index].angular_velocity[0] *= damping;
			realm.bodies.momentum[index].angular_velocity[1] *= damping;
			realm.bodies.momentum[index].angular_velocity[2] *= damping;
		}

		// Read previous impulses from contact cache.
		nudge.ContactImpulseData* contact_impulses = nudge.read_cached_impulses(
				realm.contact_cache, realm.contact_data, &temporary);

		// Setup contact constraints and apply the initial impulses.
		nudge.ContactConstraintData* contact_constraints = nudge.setup_contact_constraints(realm.active_bodies,
				realm.contact_data, realm.bodies, contact_impulses, &temporary);

		// Apply contact impulses. Increasing the number of iterations will improve stability.
		for (uint i = 0; i < iterations; ++i) {
			nudge.apply_impulses(contact_constraints, realm.bodies);
			// NOTE: Custom constraint impulses should be applied here.
		}

		// Update contact impulses.
		nudge.update_cached_impulses(contact_constraints, contact_impulses);

		// Write the updated contact impulses to the cache.
		nudge.write_cached_impulses(&realm.contact_cache, realm.contact_data, contact_impulses);

		// Move active bodies.
		nudge.advance(realm.active_bodies, realm.bodies, time_step);
	}
}

void dump() {
	// Render boxes.
	for (uint i = 0; i < realm.colliders.boxes.count; ++i) {
		uint c_body = realm.colliders.boxes.transforms[i].body;

		float[3] scale;
		float[4] rotation;
		float[3] position;

		memcpy(cast(void*) scale, cast(void*) realm.colliders.boxes.data[i].size, scale.sizeof);

		quaternion_concat(rotation, realm.bodies.transforms[c_body].rotation,
				realm.colliders.boxes.transforms[i].rotation);
		quaternion_transform(position, realm.bodies.transforms[c_body].rotation,
				realm.colliders.boxes.transforms[i].position);

		position[0] += realm.bodies.transforms[c_body].position[0];
		position[1] += realm.bodies.transforms[c_body].position[1];
		position[2] += realm.bodies.transforms[c_body].position[2];

		writefln("cube: pos(%s), rot(%s), scale(%s)", position, rotation, scale);
	}

	// Render spheres.
	for (uint i = 0; i < realm.colliders.spheres.count; ++i) {
		uint c_body = realm.colliders.spheres.transforms[i].body;

		float[3] scale;
		float[4] rotation;
		float[3] position;

		scale[0] = scale[1] = scale[2] = realm.colliders.spheres.data[i].radius;

		quaternion_concat(rotation, realm.bodies.transforms[c_body].rotation,
				realm.colliders.spheres.transforms[i].rotation);
		quaternion_transform(position, realm.bodies.transforms[c_body].rotation,
				realm.colliders.spheres.transforms[i].position);

		position[0] += realm.bodies.transforms[c_body].position[0];
		position[1] += realm.bodies.transforms[c_body].position[1];
		position[2] += realm.bodies.transforms[c_body].position[2];

		writefln("sphere: pos(%s), rot(%s), scale(%s)", position, rotation, scale);
	}
}

void main() {
	// setup
	realm = new NudgeRealm(2048, 2048, 2048);

	// allocate memory
	realm.allocate();

	// The first body is the static world.
	realm.bodies.count = 1;
	realm.bodies.idle_counters[0] = 0;
	realm.bodies.transforms[0] = NudgeRealm.identity_transform;
	memset(realm.bodies.momentum, 0, realm.bodies.momentum[0].sizeof);
	memset(realm.bodies.properties, 0, realm.bodies.properties[0].sizeof);

	// Add ground.
	{
		uint collider_ix = realm.colliders.boxes.count++;

		realm.colliders.boxes.transforms[collider_ix] = NudgeRealm.identity_transform;
		realm.colliders.boxes.transforms[collider_ix].position[1] -= 20.0f;

		realm.colliders.boxes.data[collider_ix].size[0] = 400.0f;
		realm.colliders.boxes.data[collider_ix].size[1] = 10.0f;
		realm.colliders.boxes.data[collider_ix].size[2] = 400.0f;
		realm.colliders.boxes.tags[collider_ix] = collider_ix;
	}

	// Add boxes.
	for (uint i = 0; i < 1024; ++i) {
		float sx = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;
		float sy = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;
		float sz = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;

		uint new_body = realm.add_box(8.0f * sx * sy * sz, sx, sy, sz);

		realm.bodies.transforms[new_body].position[0] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;
		realm.bodies.transforms[new_body].position[1] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 300.0f;
		realm.bodies.transforms[new_body].position[2] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;

		writefln("created box: %s, %s", realm.bodies.properties[new_body],
				realm.bodies.transforms[new_body]);
	}

	// Add spheres.
	for (uint i = 0; i < 512; ++i) {
		float s = cast(float) rand() * (1.0f / cast(float) RAND_MAX) + 0.5f;

		uint new_body = realm.add_sphere(4.18879f * s * s * s, s);

		realm.bodies.transforms[new_body].position[0] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;
		realm.bodies.transforms[new_body].position[1] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 300.0f;
		realm.bodies.transforms[new_body].position[2] += cast(float) rand() * (
				1.0f / cast(float) RAND_MAX) * 10.0f - 5.0f;

		writefln("created sphere: %s, %s", realm.bodies.properties[new_body],
				realm.bodies.transforms[new_body]);
	}

	scope (exit) {
		// cleanup
		realm.destroy();
	}

	while (true) {
		simulate();
		dump();
		Thread.sleep(dur!"msecs"(16));
	}
}
