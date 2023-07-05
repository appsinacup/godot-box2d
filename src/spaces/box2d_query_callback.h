#pragma once

#include "../bodies/box2d_collision_object.h"
#include "../box2d_type_conversions.h"
#include "box2d_direct_space_state.h"
#include <box2d/b2_fixture.h>
#include <godot_cpp/classes/physics_server2d_extension_shape_result.hpp>

class Box2DQueryCallback : public b2QueryCallback {
	PhysicsServer2DExtensionShapeResult *results;
	uint32_t collision_mask;
	bool collide_with_bodies;
	bool collide_with_areas;
	uint64_t canvas_instance_id;
	int32_t max_results;
	int hit_count = 0;

public:
	Box2DQueryCallback(PhysicsServer2DExtensionShapeResult *results,
			uint32_t collision_mask,
			bool collide_with_bodies,
			bool collide_with_areas,
			uint64_t canvas_instance_id,
			int32_t max_results);

	int32_t get_hit_count();

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	virtual bool ReportFixture(b2Fixture *fixture) override;
};
