#pragma once

#include "../bodies/box2d_collision_object.h"
#include "../box2d_type_conversions.h"
#include "box2d_direct_space_state.h"
#include <box2d/b2_fixture.h>
#include <godot_cpp/classes/physics_server2d_extension_ray_result.hpp>

class Box2DRayCastCallback : public b2RayCastCallback {
	PhysicsServer2DExtensionRayResult *result;
	uint32_t collision_mask;
	bool collide_with_bodies;
	bool collide_with_areas;
	bool hit_from_inside;
	bool hit = false;

public:
	Box2DRayCastCallback(PhysicsServer2DExtensionRayResult *result,
			uint32_t collision_mask,
			bool collide_with_bodies,
			bool collide_with_areas,
			bool hit_from_inside);

	bool get_hit();

	/// Called for each fixture found in the query. You control how the ray cast
	/// proceeds by returning a float:
	/// return -1: ignore this fixture and continue
	/// return 0: terminate the ray cast
	/// return fraction: clip the ray to this point
	/// return 1: don't clip the ray and continue
	/// @param fixture the fixture hit by the ray
	/// @param point the point of initial intersection
	/// @param normal the normal vector at the point of intersection
	/// @param fraction the fraction along the ray at the point of intersection
	/// @return -1 to filter, 0 to terminate, fraction to clip the ray for
	/// closest hit, 1 to continue
	virtual float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
			const b2Vec2 &normal, float fraction) override;
};
