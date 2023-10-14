#pragma once

#include "../bodies/box2d_collision_object.h"

#include <box2d/b2_body.h>
#include <box2d/b2_distance.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_math.h>
#include <box2d/b2_time_of_impact.h>
#include <godot_cpp/templates/vector.hpp>

using namespace godot;

class b2Vec2;
class b2Transform;
class b2Body;
class b2Shape;
class b2Sweep;
class Box2DDirectSpaceState;
class b2Fixture;
class Box2DShape;
class b2AABB;
class b2WorldManifold;

struct SweepShape {
	Box2DShape *shape;
	b2Sweep sweep;
	b2Fixture *fixture;
	b2Transform transform;
};
struct SweepTestResult {
	SweepShape sweep_shape_A;
	SweepShape sweep_shape_B;
	b2DistanceOutput distance_output;
	b2TOIOutput toi_output = { b2TOIOutput::State::e_unknown, 1 };
	b2Manifold manifold;
	b2WorldManifold world_manifold;
	bool collision = false;
	real_t safe_fraction();
	real_t unsafe_fraction(float safe_fraction);
};

class Box2DSweepTest {
public:
	static b2DistanceOutput call_b2_distance(b2Transform p_transformA, b2Shape *shapeA, int child_index_A, b2Transform p_transformB, b2Shape *shapeB, int child_index_B);
	static b2Sweep create_b2_sweep(b2Transform p_transform, b2Vec2 p_center, b2Vec2 p_motion);
	static SweepTestResult shape_cast(SweepShape p_sweep_shape_A, b2Shape *shape_A, SweepShape p_sweep_shape_B, b2Shape *shape_B, float margin);
	static Vector<b2Fixture *> query_aabb_motion(Box2DShape *p_shape, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, uint32_t p_collision_layer, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, Box2DDirectSpaceState *space_state);
	static Vector<b2Fixture *> query_aabb_motion(Vector<Box2DShape *> p_shapes, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, uint32_t p_collision_layer, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, Box2DDirectSpaceState *space_state);
	static Vector<SweepTestResult> multiple_shapes_cast(Vector<Box2DCollisionObject::Shape> p_shapes, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, bool p_collide_with_bodies, bool p_collide_with_areas, int32_t p_max_results, Vector<b2Fixture *> p_other_fixtures, Box2DDirectSpaceState *space_state);
	static Vector<SweepTestResult> multiple_shapes_cast(Box2DShape *p_shape, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, bool p_collide_with_bodies, bool p_collide_with_areas, int32_t p_max_results, Vector<b2Fixture *> p_other_fixtures, Box2DDirectSpaceState *space_state);
	static SweepTestResult closest_result_in_cast(Vector<SweepTestResult> p_results);
};
