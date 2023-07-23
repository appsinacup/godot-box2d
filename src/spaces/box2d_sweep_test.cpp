#include "box2d_sweep_test.h"

#include "../b2_user_settings.h"

#include "../box2d_type_conversions.h"
#include "../shapes/box2d_shape.h"
#include "box2d_direct_space_state.h"
#include "box2d_query_callback.h"

#include <box2d/b2_body.h>
#include <box2d/b2_chain_shape.h>
#include <box2d/b2_circle_shape.h>
#include <box2d/b2_collision.h>
#include <box2d/b2_distance.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_math.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_time_of_impact.h>

real_t SweepTestResult::safe_fraction() {
	b2Vec2 motion_normal = sweep_shape_A.sweep.c - sweep_shape_A.sweep.c0;
	float motion_length = motion_normal.Normalize();
	float unsafe_length = motion_length * toi_output.t;
	b2Vec2 separation = distance_output.pointA - sweep_shape_A.transform.p;
	float separation_distance = separation.Length();
	// Vector projection https://math.stackexchange.com/questions/108980/projecting-a-point-onto-a-vector-2d
	b2Vec2 projection = (b2Cross(separation, motion_normal)) * motion_normal;

	float safe_length = unsafe_length - projection.Length();
	float safe_fraction = safe_length / motion_length;
	if (safe_fraction <= b2_epsilon) {
		return 0;
	}
	return safe_fraction;
}
real_t SweepTestResult::unsafe_fraction(float safe_fraction) {
	if (safe_fraction <= b2_epsilon) {
		return 0;
	}
	if (safe_fraction >= toi_output.t) {
		return safe_fraction + b2_linearSlop;
	}
	return toi_output.t;
}

b2Sweep Box2DSweepTest::create_b2_sweep(b2Transform p_transform, b2Vec2 p_center, b2Vec2 p_motion) {
	b2Sweep sweep;
	sweep.a0 = p_transform.q.GetAngle();
	sweep.a = p_transform.q.GetAngle();
	sweep.localCenter = p_center;
	sweep.c0 = sweep.localCenter + p_transform.p;
	sweep.c = sweep.localCenter + p_transform.p + p_motion;
	sweep.alpha0 = 0;
	return sweep;
}

struct IntersectionManifoldResult {
	b2Manifold manifold;
	bool flipped;

	inline bool intersecting() const {
		return manifold.pointCount > 0;
	}
};
IntersectionManifoldResult _evaluate_intersection_manifold(const b2Shape *p_shapeA, const int p_child_index_A, const b2Transform &p_xfA, const b2Shape *p_shapeB, const int p_child_index_B, const b2Transform &p_xfB) {
	b2Manifold manifold{};
	bool flipped = false;

	// Convert chains to edges
	b2EdgeShape shapeA_as_edge;
	if (p_shapeA->GetType() == b2Shape::Type::e_chain) {
		static_cast<const b2ChainShape *>(p_shapeA)->GetChildEdge(&shapeA_as_edge, p_child_index_A);
		p_shapeA = &shapeA_as_edge;
	}

	b2EdgeShape shapeB_as_edge;
	if (p_shapeB->GetType() == b2Shape::Type::e_chain) {
		static_cast<const b2ChainShape *>(p_shapeB)->GetChildEdge(&shapeB_as_edge, p_child_index_B);
		p_shapeA = &shapeB_as_edge;
	}

	// This is, as far as I know, the cleanest way to implement this.
	switch (p_shapeA->GetType()) {
		case b2Shape::Type::e_circle: {
			switch (p_shapeB->GetType()) {
				case b2Shape::Type::e_circle: {
					b2CollideCircles(&manifold, static_cast<const b2CircleShape *>(p_shapeA), p_xfA, static_cast<const b2CircleShape *>(p_shapeB), p_xfB);
				} break;
				case b2Shape::Type::e_edge: {
					b2CollideEdgeAndCircle(&manifold, static_cast<const b2EdgeShape *>(p_shapeB), p_xfB, static_cast<const b2CircleShape *>(p_shapeA), p_xfA);
					flipped = true;
				} break;
				case b2Shape::Type::e_polygon: {
					b2CollidePolygonAndCircle(&manifold, static_cast<const b2PolygonShape *>(p_shapeB), p_xfB, static_cast<const b2CircleShape *>(p_shapeA), p_xfA);
					flipped = true;
				} break;
				default: {
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unandled. TODO");
				} break;
			}
		} break;
		case b2Shape::Type::e_edge: {
			switch (p_shapeB->GetType()) {
				case b2Shape::Type::e_circle: {
					b2CollideEdgeAndCircle(&manifold, static_cast<const b2EdgeShape *>(p_shapeA), p_xfA, static_cast<const b2CircleShape *>(p_shapeB), p_xfB);
				} break;
				case b2Shape::Type::e_edge: {
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "There are no contacts between two edges in Box2D. This is an invalid manifold query.");
				} break;
				case b2Shape::Type::e_polygon: {
					b2CollideEdgeAndPolygon(&manifold, static_cast<const b2EdgeShape *>(p_shapeA), p_xfA, static_cast<const b2PolygonShape *>(p_shapeB), p_xfB);
				} break;
				default: {
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unandled. TODO");
				} break;
			}
		} break;
		case b2Shape::Type::e_polygon: {
			switch (p_shapeB->GetType()) {
				case b2Shape::Type::e_circle: {
					b2CollidePolygonAndCircle(&manifold, static_cast<const b2PolygonShape *>(p_shapeA), p_xfA, static_cast<const b2CircleShape *>(p_shapeB), p_xfB);
				} break;
				case b2Shape::Type::e_edge: {
					b2CollideEdgeAndPolygon(&manifold, static_cast<const b2EdgeShape *>(p_shapeB), p_xfB, static_cast<const b2PolygonShape *>(p_shapeA), p_xfA);
					flipped = true;
				} break;
				case b2Shape::Type::e_polygon: {
					b2CollidePolygons(&manifold, static_cast<const b2PolygonShape *>(p_shapeA), p_xfA, static_cast<const b2PolygonShape *>(p_shapeB), p_xfB);
				} break;
				default: {
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unandled. TODO");
				} break;
			}
		} break;
		default: {
			ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unandled. TODO");
		} break;
	}

	return IntersectionManifoldResult{ manifold, flipped };
}

b2DistanceOutput _call_b2_distance(b2Transform p_transformA, b2Shape *shapeA, b2Transform p_transformB, b2Shape *shapeB) {
	b2DistanceOutput output;
	b2DistanceInput input;
	b2SimplexCache cache;
	cache.count = 0;
	float margin = 0.01f;
	input.proxyA.Set(shapeA, 0);
	//input.proxyA.m_radius = margin;
	input.proxyB.Set(shapeB, 0);
	//input.proxyB.m_radius = margin;
	input.transformA = p_transformA;
	input.transformB = p_transformB;
	input.useRadii = true;
	b2Distance(&output, &cache, &input);
	b2PolygonShape *polyA = (b2PolygonShape *)shapeA;
	b2PolygonShape *polyB = (b2PolygonShape *)shapeB;
	return output;
}

b2AABB get_shape_aabb(Box2DShape *shape, const b2Transform &shape_transform) {
	b2AABB aabb;
	b2AABB aabb_total;
	bool first_time = true;
	if (shape->get_b2Shape_count(false) == 0) {
		ERR_FAIL_V_MSG(aabb_total, "Cannot get aabb of empty shape.");
	}
	Transform2D identity;
	for (int i = 0; i < shape->get_b2Shape_count(false); i++) {
		b2Shape *b2_shape = (shape->get_transformed_b2Shape(i, identity, false, false));
		b2_shape->ComputeAABB(&aabb, shape_transform, 0);
		if (first_time) {
			first_time = false;
			aabb_total = aabb;
		} else {
			aabb_total.Combine(aabb);
		}
		memdelete(b2_shape);
	}
	if (!aabb.IsValid()) {
		ERR_FAIL_V_MSG(aabb_total, "aabb of shape is not valid.");
	}
	return aabb_total;
}

SweepTestResult Box2DSweepTest::shape_cast(SweepShape p_sweep_shape_A, b2Shape *shape_A, SweepShape p_sweep_shape_B, b2Shape *shape_B) {
	b2TOIInput toi_input;
	b2TOIOutput toi_output;
	b2Sweep sweep_A = p_sweep_shape_A.sweep;
	b2Sweep sweep_B = p_sweep_shape_B.sweep;
	b2Vec2 motion = sweep_A.c - sweep_A.c0;
	toi_input.tMax = 1;
	toi_input.sweepA = sweep_A;
	toi_input.sweepB = sweep_B;
	toi_input.proxyA.Set(shape_A, 0);
	toi_input.proxyB.Set(shape_B, 0);
	b2TimeOfImpact(&toi_output, &toi_input);
	b2WorldManifold manifold;
	switch (toi_output.state) {
		case b2TOIOutput::State::e_failed: // failed still gives a result, it just doesn't guarantee accuracy
		case b2TOIOutput::State::e_overlapped:
		case b2TOIOutput::State::e_touching: {
			double hit_ratio = toi_output.t;
			// move transform_A and B to end transform;
			sweep_A.GetTransform(&p_sweep_shape_A.transform, toi_output.t);
			sweep_B.GetTransform(&p_sweep_shape_B.transform, toi_output.t);
			b2DistanceOutput distance_output = _call_b2_distance(p_sweep_shape_A.transform, shape_A, p_sweep_shape_B.transform, shape_B);
			if (distance_output.distance > b2_epsilon) {
				break;
			}
			IntersectionManifoldResult intersection = _evaluate_intersection_manifold(shape_A, 0, p_sweep_shape_A.transform, shape_B, 0, p_sweep_shape_B.transform);
			b2Manifold local_manifold = intersection.manifold;

			if (!intersection.intersecting()) {
				break;
			}
			manifold.Initialize(&local_manifold, p_sweep_shape_A.transform, shape_A->m_radius, p_sweep_shape_B.transform, shape_B->m_radius);
			if (intersection.flipped) {
				manifold.normal = -manifold.normal;
			}

			const b2Vec2 normal = manifold.normal;
			if (b2Dot(normal, motion) <= FLT_EPSILON) {
				break;
			}

			return SweepTestResult{ p_sweep_shape_A, p_sweep_shape_B, distance_output, toi_output, local_manifold.pointCount, manifold, true };
		}
		default: {
		} break;
	}
	return SweepTestResult{ p_sweep_shape_A, p_sweep_shape_B, b2DistanceOutput(), toi_output, 0, manifold, false };
}
Vector<b2Fixture *> Box2DSweepTest::query_aabb_motion(Box2DShape *p_shape, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, Box2DDirectSpaceState *space_state) {
	Vector<Box2DShape *> shapes;
	shapes.append(p_shape);
	return query_aabb_motion(shapes, p_transform, p_motion, p_margin, p_collision_mask, p_collide_with_bodies, p_collide_with_areas, space_state);
}

Vector<b2Fixture *> Box2DSweepTest::query_aabb_motion(Vector<Box2DShape *> p_shapes, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, Box2DDirectSpaceState *space_state) {
	b2Vec2 motion = godot_to_box2d(p_motion);
	Box2DQueryCallback callback(space_state,
			p_collision_mask,
			p_collide_with_bodies,
			p_collide_with_areas);
	b2Transform shape_A_transform(godot_to_box2d(p_transform.get_origin()) - b2Vec2(p_margin, p_margin), b2Rot(p_transform.get_rotation()));
	Vector<b2Fixture *> shapes_result;
	for (Box2DShape *shape : p_shapes) {
		b2AABB aabb = get_shape_aabb(shape, shape_A_transform);
		aabb.Combine(get_shape_aabb(shape, b2Transform(shape_A_transform.p + motion + b2Vec2(p_margin, p_margin), shape_A_transform.q)));
		space_state->space->get_b2World()->QueryAABB(&callback, aabb);
		shapes_result.append_array(callback.get_results());
	}

	return shapes_result;
}

Vector<SweepTestResult> Box2DSweepTest::multiple_shapes_cast(Box2DShape *p_shape, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, int32_t p_max_results, Vector<b2Fixture *> p_other_fixtures, Box2DDirectSpaceState *space_state) {
	Vector<Box2DShape *> shapes;
	shapes.append(p_shape);
	return multiple_shapes_cast(shapes, p_transform, p_motion, p_margin, p_collision_mask, p_collide_with_bodies, p_collide_with_areas, p_max_results, p_other_fixtures, space_state);
}
Vector<SweepTestResult> Box2DSweepTest::multiple_shapes_cast(Vector<Box2DShape *> p_shapes, const Transform2D &p_transform, const Vector2 &p_motion, double p_margin, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, int32_t p_max_results, Vector<b2Fixture *> p_other_fixtures, Box2DDirectSpaceState *space_state) {
	Vector<SweepTestResult> results;
	if (p_max_results == 0) {
		return results;
	}
	// TODO take into consideration shape local transform
	b2Transform shape_A_transform(godot_to_box2d(p_transform.get_origin()), b2Rot(p_transform.get_rotation()));
	b2Vec2 motion = godot_to_box2d(p_motion);
	b2Sweep sweepA = create_b2_sweep(shape_A_transform, b2Vec2_zero, motion);
	Transform2D identity;
	for (int b = 0; b < p_other_fixtures.size(); b++) {
		b2Fixture *fixture_B = p_other_fixtures[b];
		b2Shape *shape_B = fixture_B->GetShape();
		b2Body *body_B = fixture_B->GetBody();
		Box2DCollisionObject *collision_object_B = body_B->GetUserData().collision_object;
		Box2DShape *box2d_shape_B = collision_object_B->get_shape(fixture_B->GetUserData().shape_idx);
		if (!body_B) {
			ERR_FAIL_V(results);
		}
		b2Sweep sweepB = create_b2_sweep(body_B->GetTransform(), body_B->GetLocalCenter(), b2Vec2_zero);
		for (Box2DShape *box2d_shape_A : p_shapes) {
			for (int i = 0; i < box2d_shape_A->get_b2Shape_count(false); i++) {
				b2Shape *shape_A = box2d_shape_A->get_transformed_b2Shape(i, identity, false, false);
				SweepShape sweep_shape_A{ box2d_shape_A, sweepA, nullptr, shape_A_transform };
				SweepShape sweep_shape_B{ box2d_shape_B, sweepB, fixture_B, body_B->GetTransform() };
				SweepTestResult output = Box2DSweepTest::shape_cast(sweep_shape_A, shape_A, sweep_shape_B, shape_B);
				if (output.collision) {
					results.append(output);
				}
				memdelete(shape_A);
			}
		}
	}
	return results;
}

SweepTestResult Box2DSweepTest::closest_result_in_cast(Vector<SweepTestResult> p_results) {
	SweepTestResult min_result;
	for (SweepTestResult result : p_results) {
		if (!min_result.collision || result.toi_output.t < min_result.toi_output.t) {
			min_result = result;
		}
	}
	return min_result;
}
