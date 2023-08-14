#pragma once

#include "../bodies/box2d_collision_object.h"
#include "../box2d_type_conversions.h"
#include "box2d_direct_space_state.h"
#include <box2d/b2_fixture.h>
#include <box2d/b2_math.h>
#include <godot_cpp/classes/physics_server2d_extension_shape_result.hpp>

using namespace godot;

class Box2DDirectSpaceState;

class Box2DQueryCallback : public b2QueryCallback {
	Box2DDirectSpaceState *space_state;
	Vector<b2Fixture *> results;
	uint32_t collision_layer;
	uint32_t collision_mask;
	bool collide_with_bodies;
	bool collide_with_areas;
	int hit_count = 0;

public:
	Box2DQueryCallback(Box2DDirectSpaceState *space_state,
			uint32_t collision_layer,
			uint32_t collision_mask,
			bool collide_with_bodies,
			bool collide_with_areas);

	Vector<b2Fixture *> get_results();

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	virtual bool ReportFixture(b2Fixture *fixture) override;
};
