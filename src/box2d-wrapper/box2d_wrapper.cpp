#include "box2d_wrapper.h"
#include "../b2_user_settings.h"
#include "../bodies/box2d_collision_object_2d.h"
#include <box2d/b2_distance.h>
#include <box2d/b2_time_of_impact.h>
#include <box2d/box2d.h>
#include <godot_cpp/templates/hash_set.hpp>

using namespace box2d;
using namespace godot;

enum class ShapeType {
	Circle = 0,
	Segment,
	Polygon,
};

struct Box2DHolder {
	HashMap<b2World *, ActiveBodyCallback> active_body_callbacks;
	HashMap<b2World *, int> active_objects;
};

Box2DHolder holder;

bool is_toi_intersected(const b2TOIOutput &output) {
	return output.state == b2TOIOutput::State::e_failed || output.state == b2TOIOutput::State::e_overlapped || output.state == b2TOIOutput::State::e_touching;
}

b2TOIOutput _time_of_impact(b2Shape *shape_A, const b2Transform &p_xfA, b2Vec2 local_centerA, const b2Vec2 &motion_A, b2Shape *shape_B, const b2Transform &p_xfB, b2Vec2 local_centerB, const b2Vec2 &motion_B) {
	b2TOIOutput output;
	b2TOIInput input;
	input.tMax = 1.0;
	input.sweepA.a0 = p_xfA.q.GetAngle();
	input.sweepA.a = input.sweepA.a0;
	input.sweepA.localCenter = b2Vec2_zero;
	input.sweepA.c0 = p_xfA.p;
	input.sweepA.c = input.sweepA.c0 + motion_A;
	input.sweepA.alpha0 = 0.0;
	input.sweepB.a0 = p_xfB.q.GetAngle();
	input.sweepB.a = input.sweepB.a0;
	input.sweepB.localCenter = b2Vec2_zero;
	input.sweepB.c0 = p_xfB.p;
	input.sweepB.c = input.sweepB.c0 + motion_B;
	input.sweepB.alpha0 = 0.0;
	for (int i = 0; i < shape_A->GetChildCount(); i++) {
		input.proxyA.Set(shape_A, i);
		for (int j = 0; j < shape_B->GetChildCount(); j++) {
			input.proxyB.Set(shape_B, j);
			b2TimeOfImpact(&output, &input);
			bool intersects = is_toi_intersected(output);
			if (intersects) {
				return output;
			}
		}
	}
	return output;
}

b2DistanceOutput _call_b2_distance(b2Transform p_transformA, b2Shape *shapeA, int child_index_A, b2Transform p_transformB, b2Shape *shapeB, int child_index_B) {
	b2DistanceOutput output;
	b2DistanceInput input;
	b2SimplexCache cache;
	cache.count = 0;
	input.proxyA.Set(shapeA, child_index_A);
	input.proxyB.Set(shapeB, child_index_B);
	input.transformA = p_transformA;
	input.transformB = p_transformB;
	input.useRadii = true;
	b2Distance(&output, &cache, &input);
	return output;
}

struct IntersectionManifoldResult {
	b2Manifold manifold;
	bool flipped;

	bool intersecting() const {
		return manifold.pointCount > 0;
	}
};

// from https://github.com/briansemrau/godot_box2d/blob/5f55923fac81386e5735573e99d908d18efec6a1/scene/2d/box2d_world.cpp#L731
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
		p_shapeB = &shapeB_as_edge;
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
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unexpected shape type.");
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
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unexpected shape type.");
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
					ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unexpected shape type.");
				} break;
			}
		} break;
		default: {
			ERR_FAIL_V_MSG((IntersectionManifoldResult{ manifold, flipped }), "Unexpected shape type.");
		} break;
	}

	return IntersectionManifoldResult{ manifold, flipped };
}

struct IntersectionResult {
	b2WorldManifold world_manifold;
	b2Manifold manifold;
	b2DistanceOutput distance_output;
};

IntersectionResult _intersect_shape(b2Shape *shape_A, const b2Transform &p_xfA, b2Shape *shape_B, const b2Transform &p_xfB, real_t margin) {
	b2WorldManifold manifold;
	b2DistanceOutput distance_output;
	for (int i = 0; i < shape_A->GetChildCount(); i++) {
		for (int j = 0; j < shape_B->GetChildCount(); j++) {
			distance_output.distance = margin + 1.0;
			distance_output = _call_b2_distance(p_xfA, shape_A, i, p_xfB, shape_B, j);
			if (distance_output.distance > margin + CMP_EPSILON) {
				break;
			}
			IntersectionManifoldResult intersection = _evaluate_intersection_manifold(shape_A, i, p_xfA, shape_B, j, p_xfB);
			b2Manifold local_manifold = intersection.manifold;
			if (!intersection.intersecting()) {
				//break;
			}
			if (intersection.flipped) {
				manifold.Initialize(&local_manifold, p_xfB, shape_B->m_radius, p_xfA, shape_A->m_radius);
				manifold.normal = -manifold.normal;
			} else {
				manifold.Initialize(&local_manifold, p_xfA, shape_A->m_radius, p_xfB, shape_B->m_radius);
			}

			//if (b2Dot(manifold.normal, motion) <= FLT_EPSILON && !Vector2(motion.x, motion.y).is_zero_approx()) {
			//	break;
			//}

			// TODO return all results?
			return IntersectionResult{
				manifold,
				local_manifold,
				distance_output
			};
		}
	}
	return IntersectionResult{
		manifold,
		b2Manifold{},
		distance_output,
	};
}

bool box2d::are_handles_equal(b2World *handle1, b2World *handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(b2Body *handle1, b2Body *handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(FixtureHandle handle1, FixtureHandle handle2) {
	if (handle1.count != handle2.count) {
		return false;
	}
	for (int i = 0; i < handle1.count; i++) {
		if (handle1.handles[i] != handle2.handles[i]) {
			return false;
		}
	}
	return true;
}
bool box2d::are_handles_equal(ShapeHandle handle1, ShapeHandle handle2) {
	if (handle1.count != handle2.count) {
		return false;
	}
	for (int i = 0; i < handle1.count; i++) {
		if (handle1.handles[i] != handle2.handles[i]) {
			return false;
		}
	}
	return true;
}
bool box2d::are_handles_equal(b2Joint *handle1, b2Joint *handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(b2Fixture *handle1, b2Fixture *handle2) {
	return handle1 == handle2;
}

void box2d::body_add_force(b2World *world_handle, b2Body *body_handle, const b2Vec2 force) {
	body_handle->GetUserData().constant_force += b2Vec2_to_Vector2(force);
}

void box2d::body_add_torque(b2World *world_handle, b2Body *body_handle, real_t torque) {
	body_handle->GetUserData().constant_torque += torque;
}

void box2d::body_apply_impulse(b2World *world_handle, b2Body *body_handle, const b2Vec2 impulse) {
	body_handle->ApplyLinearImpulseToCenter(impulse, true);
}

void box2d::body_apply_impulse_at_point(b2World *world_handle,
		b2Body *body_handle,
		const b2Vec2 impulse,
		const b2Vec2 point) {
	body_handle->ApplyLinearImpulse(impulse, point, true);
}

void box2d::body_apply_torque_impulse(b2World *world_handle, b2Body *body_handle, real_t torque_impulse) {
	body_handle->ApplyAngularImpulse(torque_impulse, true);
}

void box2d::body_change_mode(b2World *world_handle, b2Body *body_handle, b2BodyType body_type, bool wakeup, bool fixed_rotation) {
	body_handle->SetType(body_type);
	body_handle->SetFixedRotation(fixed_rotation);
}

b2Body *box2d::body_create(b2World *world_handle,
		const b2Vec2 pos,
		real_t rot,
		const b2BodyUserData &user_data,
		b2BodyType body_type) {
	b2BodyDef body_def;
	body_def.position = pos;
	body_def.angle = rot;
	body_def.userData = user_data;
	body_def.type = body_type;
	return world_handle->CreateBody(&body_def);
}

void box2d::body_destroy(b2World *world_handle, b2Body *body_handle) {
	world_handle->DestroyBody(body_handle);
}

void box2d::body_force_sleep(b2World *world_handle, b2Body *body_handle) {
	if (body_handle->IsSleepingAllowed()) {
		body_handle->SetAwake(false);
	}
}

real_t box2d::body_get_angle(b2World *world_handle, b2Body *body_handle) {
	return body_handle->GetAngle();
}

real_t box2d::body_get_angular_velocity(b2World *world_handle, b2Body *body_handle) {
	if (body_handle->GetType() == b2_kinematicBody) {
		return body_handle->GetUserData().old_angular_velocity;
	}
	return body_handle->GetAngularVelocity();
}

b2Vec2 box2d::body_get_constant_force(b2World *world_handle, b2Body *body_handle) {
	return Vector2_to_b2Vec2(body_handle->GetUserData().constant_force);
}

real_t box2d::body_get_constant_torque(b2World *world_handle, b2Body *body_handle) {
	return body_handle->GetUserData().constant_torque;
}

b2Vec2 box2d::body_get_linear_velocity(b2World *world_handle, b2Body *body_handle) {
	if (body_handle->GetType() == b2_kinematicBody) {
		return Vector2_to_b2Vec2(body_handle->GetUserData().old_linear_velocity);
	}
	return body_handle->GetLinearVelocity();
}

b2Vec2 box2d::body_get_position(b2World *world_handle, b2Body *body_handle) {
	return body_handle->GetPosition();
}

bool box2d::body_is_ccd_enabled(b2World *world_handle, b2Body *body_handle) {
	return body_handle->IsBullet();
}

void box2d::body_reset_forces(b2World *world_handle, b2Body *body_handle) {
	body_handle->GetUserData().constant_force = Vector2();
}

void box2d::body_reset_torques(b2World *world_handle, b2Body *body_handle) {
	body_handle->GetUserData().constant_torque = 0.0;
}

void box2d::body_set_angular_damping(b2World *world_handle, b2Body *body_handle, real_t angular_damping) {
	body_handle->SetAngularDamping(angular_damping);
}

void box2d::body_set_angular_velocity(b2World *world_handle, b2Body *body_handle, real_t vel) {
	body_handle->SetAngularVelocity(vel);
}

void box2d::body_set_can_sleep(b2World *world_handle, b2Body *body_handle, bool can_sleep) {
	body_handle->SetSleepingAllowed(can_sleep);
}

void box2d::body_set_ccd_enabled(b2World *world_handle, b2Body *body_handle, bool enable) {
	body_handle->SetBullet(enable);
}

void box2d::body_set_gravity_scale(b2World *world_handle,
		b2Body *body_handle,
		real_t gravity_scale,
		bool wake_up) {
	body_handle->SetGravityScale(gravity_scale);
}

void box2d::body_set_linear_damping(b2World *world_handle, b2Body *body_handle, real_t linear_damping) {
	body_handle->SetLinearDamping(linear_damping);
}

void box2d::body_set_linear_velocity(b2World *world_handle, b2Body *body_handle, const b2Vec2 vel) {
	if (body_handle->GetType() == b2_kinematicBody) {
		// for kinematic setting velocity is like moving the object, we want it to happen all the time
		body_handle->SetLinearVelocity(vel + body_handle->GetLinearVelocity());
	} else {
		body_handle->SetLinearVelocity(vel);
	}
}

void box2d::body_set_mass_properties(b2World *world_handle,
		b2Body *body_handle,
		real_t mass,
		real_t inertia,
		const b2Vec2 local_com,
		bool wake_up,
		bool force_update) {
	b2MassData mass_data{
		mass,
		local_com,
		inertia
	};
	body_handle->SetMassData(&mass_data);
}

void box2d::body_set_transform(b2World *world_handle,
		b2Body *body_handle,
		const b2Vec2 pos,
		real_t rot,
		bool wake_up,
		real_t step) {
	b2Vec2 new_pos = (pos - body_handle->GetPosition());
	new_pos.x /= step;
	new_pos.y /= step;
	if (body_handle->GetType() == b2BodyType::b2_kinematicBody) {
		real_t new_rot1 = (rot - body_handle->GetAngle()) / step;
		real_t new_rot2 = -(2.0 * b2_pi - rot + body_handle->GetAngle()) / step;
		real_t new_rot = new_rot1;
		if (ABS(new_rot2) < ABS(new_rot1)) {
			new_rot = new_rot2;
		}
		body_handle->SetLinearVelocity(new_pos);
		body_handle->SetAngularVelocity(new_rot);
	} else {
		body_handle->SetTransform(pos, rot);
		if (body_handle->IsSleepingAllowed()) {
			body_handle->SetAwake(wake_up);
		} else {
			body_handle->SetAwake(true);
		}
	}
	// also update collider transform with new values
}

void box2d::body_update_material(b2World *world_handle, b2Body *body_handle, const Material *mat) {
	for (b2Fixture *fixture = body_handle->GetFixtureList(); fixture != nullptr; fixture = fixture->GetNext()) {
		fixture->SetFriction(mat->friction);
		fixture->SetRestitution(mat->restitution);
	}
}

void box2d::body_wake_up(b2World *world_handle, b2Body *body_handle, bool strong) {
	body_handle->SetAwake(true);
}

FixtureHandle box2d::collider_create_sensor(b2World *world_handle,
		ShapeHandle shape_handle,
		b2Body *body_handle,
		b2FixtureUserData user_data) {
	FixtureHandle fixture_handle{
		(b2Fixture **)memalloc((shape_handle.count) * sizeof(b2Fixture *)),
		shape_handle.count
	};
	b2MassData mass_data = body_handle->GetMassData();
	for (int i = 0; i < shape_handle.count; i++) {
		b2FixtureDef fixture_def;
		fixture_def.shape = shape_handle.handles[i];
		fixture_def.density = 1.0;
		fixture_def.isSensor = true;
		fixture_def.userData = user_data;
		b2Fixture *fixture = body_handle->CreateFixture(&fixture_def);
		fixture_handle.handles[i] = fixture;
	}
	body_handle->SetMassData(&mass_data);
	return fixture_handle;
}

FixtureHandle box2d::collider_create_solid(b2World *world_handle,
		ShapeHandle shape_handle,
		const Material *mat,
		b2Body *body_handle,
		b2FixtureUserData user_data) {
	FixtureHandle fixture_handle{
		(b2Fixture **)memalloc((shape_handle.count) * sizeof(b2Fixture *)),
		shape_handle.count
	};
	b2MassData mass_data = body_handle->GetMassData();
	for (int i = 0; i < shape_handle.count; i++) {
		b2FixtureDef fixture_def;
		fixture_def.shape = shape_handle.handles[i];
		fixture_def.density = 1.0;
		fixture_def.restitution = mat->restitution;
		fixture_def.friction = mat->friction;
		fixture_def.isSensor = false;
		fixture_def.userData = user_data;
		b2Fixture *fixture = body_handle->CreateFixture(&fixture_def);
		fixture_handle.handles[i] = fixture;
	}
	body_handle->SetMassData(&mass_data);
	return fixture_handle;
}

void box2d::collider_destroy(b2World *world_handle, FixtureHandle handle) {
	ERR_FAIL_COND(!is_handle_valid(handle));
	b2Body *body = handle.handles[0]->GetBody();
	if (!is_handle_valid(body)) {
		// already destroyed
		return;
	}
	for (b2Fixture *fixture = body->GetFixtureList(); fixture != nullptr; fixture = fixture->GetNext()) {
		for (int i = 0; i < handle.count; i++) {
			if (fixture == handle.handles[i]) {
				body->DestroyFixture(fixture);
			}
		}
	}
}

b2Vec2 box2d::Vector2_to_b2Vec2(Vector2 vec) {
	return b2Vec2(vec.x, vec.y);
}

b2Transform Transform2D_to_b2Transform(Transform2D transform) {
	return b2Transform(Vector2_to_b2Vec2(transform.get_origin()), b2Rot(transform.get_rotation()));
}

Transform2D b2Transform_to_Transform2D(b2Transform transform) {
	return Transform2D(transform.q.GetAngle(), b2Vec2_to_Vector2(transform.p));
}

Vector2 box2d::b2Vec2_to_Vector2(b2Vec2 vec) {
	return Vector2(vec.x, vec.y);
}

b2Vec2 xform_b2Vec2(b2Vec2 vec, Transform2D transform) {
	return Vector2_to_b2Vec2(transform.xform(b2Vec2_to_Vector2(vec)));
}

void box2d::collider_set_transform(b2World *world_handle, FixtureHandle handles, ShapeInfo shape_info) {
	for (int i = 0; i < handles.count; i++) {
		b2Fixture *handle = handles.handles[i];
		handle->GetUserData().transform = shape_info.transform;
		ERR_FAIL_COND(!handle);
		ERR_FAIL_COND(!is_handle_valid(shape_info.handle));
		ERR_FAIL_COND(handles.count != shape_info.handle.count);
		b2Shape *shape_template = shape_info.handle.handles[i];
		b2Shape *shape = handle->GetShape();
		ERR_FAIL_COND(!shape);
		b2Shape::Type shape_type = shape->GetType();
		Transform2D transform = shape_info.transform;
		shape_info.body_transform.set_origin(Vector2());
		shape_info.body_transform.set_rotation(0.0);
		transform = shape_info.body_transform * transform;
		switch (shape_type) {
			case b2Shape::Type::e_circle: {
				b2CircleShape *circle_shape_template = (b2CircleShape *)shape_template;
				b2CircleShape *circle_shape = (b2CircleShape *)shape;
				circle_shape->m_p = xform_b2Vec2(circle_shape_template->m_p, transform);
				if (transform.get_scale().x == transform.get_scale().y) {
					circle_shape->m_radius = transform.get_scale().x * circle_shape_template->m_radius;
				}
			} break;
			case b2Shape::Type::e_chain: {
				b2ChainShape *chain_shape_template = (b2ChainShape *)shape_template;
				b2ChainShape *chain_shape = (b2ChainShape *)shape;
				b2Vec2 new_vertices[b2_maxPolygonVertices];
				for (int i = 0; i < chain_shape->m_count; i++) {
					new_vertices[i] = chain_shape_template->m_vertices[i];
					new_vertices[i] = xform_b2Vec2(new_vertices[i], transform);
				}
				int count = chain_shape->m_count;
				chain_shape->m_count = 0;
				chain_shape->CreateLoop(new_vertices, chain_shape->m_count);
				if (!chain_shape->m_count) {
					chain_shape->m_count = count;
					ERR_FAIL_MSG("Cannot update vertices");
				}
			} break;
			case b2Shape::Type::e_polygon: {
				b2PolygonShape *polygon_shape_template = (b2PolygonShape *)shape_template;
				b2PolygonShape *polygon_shape = (b2PolygonShape *)shape;
				b2Vec2 new_vertices[b2_maxPolygonVertices];
				b2Hull hull;
				for (int i = 0; i < polygon_shape->m_count; i++) {
					new_vertices[i] = polygon_shape_template->m_vertices[i];
					new_vertices[i] = xform_b2Vec2(new_vertices[i], transform);
					hull.points[i] = new_vertices[i];
				}
				hull.count = polygon_shape->m_count;
				polygon_shape->Set(hull);
				if (!polygon_shape->m_count) {
					ERR_FAIL_MSG("Cannot update vertices");
				}
			} break;
			default: {
				ERR_FAIL_MSG("Invalid Shape Type");
			}
		}
	}
}

Transform2D box2d::collider_get_transform(b2World *world_handle, FixtureHandle handle) {
	ERR_FAIL_COND_V(!is_handle_valid(handle), Transform2D());
	return handle.handles[0]->GetUserData().transform;
}

Transform2D box2d::collider_get_transform(b2World *world_handle, b2Fixture *handle) {
	return handle->GetUserData().transform;
}

Material box2d::default_material() {
	return Material{};
}

QueryExcludedInfo box2d::default_query_excluded_info() {
	return QueryExcludedInfo{ 0, 0, nullptr, 0, 0 };
}

WorldSettings box2d::default_world_settings() {
	return WorldSettings{};
}

class AABBQueryCallback : public b2QueryCallback {
public:
	int count = 0;
	bool test_point = false;
	b2Vec2 point;
	b2World *world;
	bool collide_with_body;
	bool collide_with_area;
	PointHitInfo *hit_info_array;
	size_t hit_info_length;
	QueryHandleExcludedCallback handle_excluded_callback;
	const QueryExcludedInfo *handle_excluded_info;

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	virtual bool ReportFixture(b2Fixture *fixture) override {
		if (fixture->IsSensor() && !collide_with_area) {
			return true;
		}
		if (!fixture->IsSensor() && !collide_with_body) {
			return true;
		}
		if (!handle_excluded_callback(world, fixture, fixture->GetUserData(), handle_excluded_info)) {
			hit_info_array[count++] = PointHitInfo{
				fixture,
				fixture->GetUserData()
			};
		}
		if (test_point && fixture->GetBody()) {
			if (!fixture->GetShape()->TestPoint(fixture->GetBody()->GetTransform(), point)) {
				return true;
			}
		}
		return count < hit_info_length;
	}
};

size_t box2d::intersect_aabb(b2World *world_handle,
		const b2Vec2 aabb_min,
		const b2Vec2 aabb_max,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	AABBQueryCallback callback;
	callback.world = world_handle;
	callback.collide_with_body = collide_with_body;
	callback.collide_with_area = collide_with_area;
	callback.hit_info_array = hit_info_array;
	callback.hit_info_length = hit_info_length;
	callback.handle_excluded_callback = handle_excluded_callback;
	callback.handle_excluded_info = handle_excluded_info;
	world_handle->QueryAABB(&callback, b2AABB{ aabb_min, aabb_max });
	return callback.count;
}

size_t box2d::intersect_point(b2World *world_handle,
		const b2Vec2 position,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	AABBQueryCallback callback;
	callback.world = world_handle;
	callback.collide_with_body = collide_with_body;
	callback.collide_with_area = collide_with_area;
	callback.hit_info_array = hit_info_array;
	callback.hit_info_length = hit_info_length;
	callback.handle_excluded_callback = handle_excluded_callback;
	callback.handle_excluded_info = handle_excluded_info;
	callback.test_point = true;
	callback.point = position;
	world_handle->QueryAABB(&callback, b2AABB{ position - b2Vec2(1, 1), position + b2Vec2(1, 1) });
	return callback.count;
}

class RayCastQueryCallback : public b2RayCastCallback {
public:
	int count = 0;
	b2World *world;
	bool collide_with_body;
	bool collide_with_area;
	RayHitInfo *hit_info_array;
	QueryHandleExcludedCallback handle_excluded_callback;
	const QueryExcludedInfo *handle_excluded_info;

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
			const b2Vec2 &normal, float fraction) override {
		if (fixture->IsSensor() && !collide_with_area) {
			return -1;
		}
		if (!fixture->IsSensor() && !collide_with_body) {
			return -1;
		}
		if (!handle_excluded_callback(world, fixture, fixture->GetUserData(), handle_excluded_info)) {
			hit_info_array[count++] = RayHitInfo{
				point,
				normal,
				fixture,
				fixture->GetUserData()
			};
		}
		return 1;
	}
};

bool box2d::intersect_ray(b2World *world_handle,
		const b2Vec2 from,
		const b2Vec2 dir,
		real_t length,
		bool collide_with_body,
		bool collide_with_area,
		bool hit_from_inside,
		RayHitInfo *hit_info,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	RayCastQueryCallback callback;
	callback.world = world_handle;
	callback.collide_with_body = collide_with_body;
	callback.collide_with_area = collide_with_area;
	callback.hit_info_array = hit_info;
	callback.handle_excluded_callback = handle_excluded_callback;
	callback.handle_excluded_info = handle_excluded_info;
	world_handle->RayCast(&callback, from, from + length * dir);
	if (callback.count) {
		return callback.count;
	}
	if (hit_from_inside) {
		world_handle->RayCast(&callback, from + length * dir, from);
	}
	return callback.count;
}

b2World *box2d::invalid_world_handle() {
	return nullptr;
}
FixtureHandle box2d::invalid_fixture_handle() {
	return FixtureHandle{
		nullptr,
		0
	};
}
b2Body *box2d::invalid_body_handle() {
	return nullptr;
}
ShapeHandle box2d::invalid_shape_handle() {
	return ShapeHandle{
		nullptr,
		0
	};
}
b2Joint *box2d::invalid_joint_handle() {
	return nullptr;
}

b2FixtureUserData box2d::invalid_fixture_user_data() {
	return b2FixtureUserData{};
}
b2BodyUserData box2d::invalid_body_user_data() {
	return b2BodyUserData{};
}
bool box2d::is_user_data_valid(b2FixtureUserData user_data) {
	return user_data.collision_object != nullptr;
}
bool box2d::is_user_data_valid(b2BodyUserData user_data) {
	return user_data.collision_object != nullptr;
}

bool box2d::is_handle_valid(FixtureHandle handle) {
	return handle.count > 0 || handle.handles != nullptr;
}
bool box2d::is_handle_valid(b2World *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(b2Fixture *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(ShapeHandle handle) {
	return handle.count > 0 || handle.handles != nullptr;
}
bool box2d::is_handle_valid(b2Body *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(b2Joint *handle) {
	return handle != nullptr;
}

void box2d::joint_set_disable_collision(b2Joint *joint_handle,
		bool disable_collision) {
}

void box2d::joint_change_revolute_params(b2World *world_handle,
		b2Joint *joint_handle,
		real_t angular_limit_lower,
		real_t angular_limit_upper,
		bool angular_limit_enabled,
		real_t motor_target_velocity,
		bool motor_enabled) {
	b2RevoluteJoint *joint = (b2RevoluteJoint *)joint_handle;
	joint->SetLimits(angular_limit_lower, angular_limit_upper);
	joint->EnableLimit(angular_limit_enabled);
	joint->SetMotorSpeed(motor_target_velocity);
	joint->EnableMotor(motor_enabled);
}

b2Joint *box2d::joint_create_prismatic(b2World *world_handle,
		b2Body *body_handle_1,
		b2Body *body_handle_2,
		const b2Vec2 axis,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		const b2Vec2 limits,
		bool disable_collision) {
	b2PrismaticJointDef joint_def;
	joint_def.Initialize(body_handle_1, body_handle_2, anchor_1, axis);
	joint_def.localAnchorA = anchor_1;
	joint_def.localAnchorB = anchor_2;
	joint_def.localAxisA = axis;
	joint_def.lowerTranslation = 0;
	joint_def.upperTranslation = limits.x + limits.y * 2;
	joint_def.enableLimit = true;
	joint_def.collideConnected = !disable_collision;
	return world_handle->CreateJoint(&joint_def);
}

b2Joint *box2d::joint_create_revolute(b2World *world_handle,
		b2Body *body_handle_1,
		b2Body *body_handle_2,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		real_t angular_limit_lower,
		real_t angular_limit_upper,
		bool angular_limit_enabled,
		real_t motor_target_velocity,
		bool motor_enabled,
		bool disable_collision) {
	b2RevoluteJointDef joint_def;
	joint_def.Initialize(body_handle_1, body_handle_2, anchor_1);
	joint_def.localAnchorA = anchor_1;
	joint_def.localAnchorB = anchor_2;
	joint_def.enableLimit = angular_limit_enabled;
	joint_def.enableMotor = motor_enabled;
	joint_def.lowerAngle = angular_limit_lower;
	joint_def.upperAngle = angular_limit_upper;
	joint_def.motorSpeed = motor_target_velocity;
	joint_def.maxMotorTorque = 100000.0;
	joint_def.collideConnected = !disable_collision;
	return world_handle->CreateJoint(&joint_def);
}

void box2d::joint_change_distance_joint(b2World *world_handle,
		b2Joint *joint_handle,
		real_t rest_length,
		real_t stiffness,
		real_t damping) {
	b2DistanceJoint *joint = (b2DistanceJoint *)joint_handle;
	joint->SetDamping(damping);
	joint->SetStiffness(stiffness);
	joint->SetLength(rest_length);
}

b2Joint *box2d::joint_create_distance_joint(b2World *world_handle,
		b2Body *body_handle_1,
		b2Body *body_handle_2,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		real_t rest_length,
		real_t max_length,
		real_t stiffness,
		real_t damping,
		bool disable_collision) {
	b2DistanceJointDef joint_def;
	joint_def.bodyA = body_handle_1;
	joint_def.bodyB = body_handle_2;
	joint_def.localAnchorA = anchor_1;
	joint_def.localAnchorB = anchor_2;
	joint_def.length = rest_length;
	joint_def.maxLength = max_length;
	joint_def.minLength = 0.0;
	joint_def.stiffness = stiffness;
	joint_def.damping = damping;
	joint_def.collideConnected = !disable_collision;
	return world_handle->CreateJoint(&joint_def);
}

void box2d::joint_destroy(b2World *world_handle, b2Joint *joint_handle) {
	world_handle->DestroyJoint(joint_handle);
}

ShapeCastResult box2d::shape_casting(b2World *world_handle,
		const b2Vec2 motion,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info,
		double margin) {
	for (int i = 0; i < shape_info.handle.count; i++) {
		shape_info.handle.handles[i]->m_radius += margin;
		b2AABB shape_aabb;
		b2AABB shape_aabb_motion;
		b2AABB shape_aabb_total;
		bool first_time = true;
		b2Transform shape_transform = Transform2D_to_b2Transform(shape_info.body_transform * shape_info.transform);
		b2Transform shape_transform_motion = shape_transform;
		shape_transform_motion.p += motion;
		for (int j = 0; j < shape_info.handle.handles[i]->GetChildCount(); j++) {
			shape_info.handle.handles[i]->ComputeAABB(&shape_aabb, shape_transform, j);
			shape_info.handle.handles[i]->ComputeAABB(&shape_aabb_motion, shape_transform_motion, j);
			shape_aabb.Combine(shape_aabb_motion);
			if (first_time) {
				shape_aabb_total = shape_aabb;
			} else {
				shape_aabb_total.Combine(shape_aabb);
			}
		}
		shape_aabb.lowerBound -= b2Vec2(margin, margin);
		shape_aabb.upperBound += b2Vec2(margin, margin);
		const size_t hit_info_length = 10;
		PointHitInfo hit_info_array[hit_info_length];
		size_t count = intersect_aabb(world_handle, shape_aabb_total.lowerBound, shape_aabb_total.upperBound, collide_with_body, collide_with_area, hit_info_array, hit_info_length, handle_excluded_callback, handle_excluded_info);
		ShapeCastResult result;
		result.collided = false;
		for (int j = 0; j < count; j++) {
			b2Body *other_body = hit_info_array[j].collider->GetBody();
			Transform2D other_transform = collider_get_transform(world_handle, hit_info_array[j].collider);
			// TODO take into consideration scale here?
			Transform2D other_body_transform = b2Transform_to_Transform2D(other_body->GetTransform());

			other_transform = other_body_transform * other_transform;
			b2Transform other_body_transform_b2 = Transform2D_to_b2Transform(other_transform);
			b2TOIOutput toi_output = _time_of_impact(shape_info.handle.handles[i], shape_transform, b2Vec2_zero, motion, hit_info_array[j].collider->GetShape(), other_body_transform_b2, other_body->GetLocalCenter(), b2Vec2_zero);
			if (is_toi_intersected(toi_output)) {
				b2Transform shape_transform_moved = shape_transform;
				shape_transform_moved.p += toi_output.t * motion;
				IntersectionResult intersection_result = _intersect_shape(shape_info.handle.handles[i], shape_transform_moved, hit_info_array[j].collider->GetShape(), other_body_transform_b2, margin);
				if (intersection_result.distance_output.distance < margin + CMP_EPSILON) {
					//intersection_result.world_manifold.normal.Normalize();
					result.collided = true;
					result.collider = hit_info_array[j].collider;
					result.normal1 = intersection_result.world_manifold.normal;
					result.normal2 = -intersection_result.world_manifold.normal;
					result.toi = toi_output.t;
					result.user_data = result.collider->GetUserData();
					result.witness1 = intersection_result.distance_output.pointA;
					result.witness2 = intersection_result.distance_output.pointB;
					break;
				}
			}
		}
		shape_info.handle.handles[i]->m_radius -= margin;
		if (result.collided) {
			return result;
		}
	}
	return ShapeCastResult{
		false
	};
}

ShapeCollideResult box2d::shape_collide(const b2Vec2 motion1,
		ShapeInfo shape_info1,
		const b2Vec2 motion2,
		ShapeInfo shape_info2) {
	for (int i = 0; i < shape_info1.handle.count; i++) {
		for (int j = 0; j < shape_info2.handle.count; j++) {
			b2Transform transformA = Transform2D_to_b2Transform(shape_info1.body_transform * shape_info1.transform);
			b2Transform transformB = Transform2D_to_b2Transform(shape_info2.body_transform * shape_info2.transform);
			b2TOIOutput toi_output = _time_of_impact(shape_info1.handle.handles[i],
					transformA,
					b2Vec2_zero,
					motion1,
					shape_info2.handle.handles[i],
					transformB,
					b2Vec2_zero,
					motion2);
			if (!is_toi_intersected(toi_output)) {
				continue;
			}
			ShapeCollideResult result;
			result.collided = true;
			transformA.p = transformA.p + toi_output.t * motion1;
			transformB.p = transformB.p + toi_output.t * motion2;
			b2DistanceOutput distance_output = _call_b2_distance(transformA, shape_info1.handle.handles[i], 0, transformB, shape_info2.handle.handles[j], 0);
			result.witness1 = distance_output.pointA;
			result.witness2 = distance_output.pointB;
			return result;
		}
	}
	return ShapeCollideResult{
		false
	};
}

ShapeHandle box2d::shape_create_box(const b2Vec2 size) {
	ERR_FAIL_COND_V(size.x < CMP_EPSILON, invalid_shape_handle());
	ERR_FAIL_COND_V(size.y < CMP_EPSILON, invalid_shape_handle());
	b2Shape **shapes = (b2Shape **)memalloc((1) * sizeof(b2Shape *));
	b2PolygonShape *shape = memnew(b2PolygonShape);
	shape->SetAsBox(size.x * 0.5, size.y * 0.5);
	shapes[0] = shape;
	return ShapeHandle{
		shapes,
		1
	};
}

ShapeHandle box2d::shape_create_capsule(real_t half_height, real_t radius) {
	ERR_FAIL_COND_V(radius < CMP_EPSILON, invalid_shape_handle());
	ERR_FAIL_COND_V(half_height < CMP_EPSILON, invalid_shape_handle());
	ERR_FAIL_COND_V(half_height < radius, invalid_shape_handle());
	real_t half_height_circle = half_height - radius;
	ShapeHandle top_circle = shape_create_circle(radius, { 0.0, -half_height_circle });
	ERR_FAIL_COND_V(!is_handle_valid(top_circle), invalid_shape_handle());
	if (half_height - radius == 0.0) {
		return top_circle;
	}
	ShapeHandle bottom_circle = shape_create_circle(radius, { 0.0, half_height_circle });
	ERR_FAIL_COND_V(!is_handle_valid(bottom_circle), invalid_shape_handle());
	ShapeHandle square = shape_create_box({ radius * 2, half_height * 2 - radius * 2 });
	ERR_FAIL_COND_V(!is_handle_valid(square), invalid_shape_handle());
	b2Shape **shapes = (b2Shape **)memalloc((3) * sizeof(b2Shape *));
	// TODO fix this leak.
	shapes[0] = top_circle.handles[0];
	shapes[1] = bottom_circle.handles[0];
	shapes[2] = square.handles[0];
	return ShapeHandle{
		shapes,
		3
	};
}

ShapeHandle box2d::shape_create_circle(real_t radius, b2Vec2 pos) {
	ERR_FAIL_COND_V(radius < CMP_EPSILON, invalid_shape_handle());
	b2Shape **shapes = (b2Shape **)memalloc((1) * sizeof(b2Shape *));
	b2CircleShape *shape = memnew(b2CircleShape);
	shape->m_radius = radius;
	shape->m_p = pos;
	shapes[0] = shape;
	return ShapeHandle{
		shapes,
		1
	};
}

ShapeHandle box2d::shape_create_concave_polyline(const b2Vec2 *points, size_t point_count) {
	b2Shape **shapes = (b2Shape **)memalloc((1) * sizeof(b2Shape *));
	b2ChainShape *shape = memnew(b2ChainShape);
	shape->CreateLoop(points, point_count);
	shapes[0] = shape;
	ERR_FAIL_COND_V(!shape->m_count, invalid_shape_handle());
	return ShapeHandle{
		shapes,
		1
	};
}

ShapeHandle box2d::shape_create_convex_polyline(const b2Vec2 *points, size_t point_count) {
	b2Shape **shapes = (b2Shape **)memalloc((1) * sizeof(b2Shape *));
	b2PolygonShape *shape = memnew(b2PolygonShape);
	shape->Set(points, point_count);
	if (shape->m_count == 0) {
		ERR_FAIL_V(invalid_shape_handle());
	}
	shapes[0] = shape;
	return ShapeHandle{
		shapes,
		1
	};
}

ShapeHandle box2d::shape_create_halfspace(const b2Vec2 normal, real_t distance) {
	real_t world_size = 1000000.0;
	b2Shape **shapes = (b2Shape **)memalloc((1) * sizeof(b2Shape *));
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 points[4];
	b2Vec2 right(normal.y, -normal.x);
	b2Vec2 left(-right);
	left *= world_size;
	right *= world_size;
	left = left + distance * normal;
	right = right + distance * normal;
	points[0] = left;
	points[1] = right;
	points[2] = right - world_size * normal;
	points[3] = right - world_size * normal;
	bool result = shape->Set(points, 4);
	shapes[0] = shape;
	ERR_FAIL_COND_V(!result, invalid_shape_handle());
	return ShapeHandle{
		shapes,
		1
	};
}

void box2d::shape_destroy(ShapeHandle shape_handle) {
	memfree(shape_handle.handles);
}

ContactResult box2d::shapes_contact(b2World *world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		real_t margin) {
	for (int i = 0; i < shape_info1.handle.count; i++) {
		for (int j = 0; j < shape_info2.handle.count; j++) {
			b2Transform transform_A = Transform2D_to_b2Transform(shape_info1.body_transform * shape_info1.transform);
			b2Transform transform_B = Transform2D_to_b2Transform(shape_info2.body_transform * shape_info2.transform);
			IntersectionResult intersection_result = _intersect_shape(shape_info1.handle.handles[i],
					transform_A,
					shape_info2.handle.handles[j],
					transform_B,
					margin);
			if (intersection_result.distance_output.distance > margin) {
				continue;
			}
			b2Vec2 point_A = intersection_result.distance_output.pointA;
			b2Vec2 point_B = intersection_result.distance_output.pointB;
			b2Vec2 normal = -point_A + point_B;
			real_t dist = (normal).Length();
			// manually compute normal
			if (dist != 0) {
				normal = b2Vec2(normal.x / dist, normal.y / dist);
			} else {
				normal = (transform_A.p - transform_B.p);
				normal.Normalize();
			}
			//normal = -intersection_result.world_manifold.normal;

			if (intersection_result.world_manifold.normal.x != 0.0 && intersection_result.world_manifold.normal.y != 0.0) {
				//normal = -intersection_result.world_manifold.normal;
				//normal.Normalize();
			}
			if (intersection_result.distance_output.distance <= 0.0) {
				// Still not perfect, because of polygon skin
				point_A -= margin * 1.07 * normal;
			} else {
				point_A += margin * 1.07 * normal;
			}
			normal = -point_A + point_B;
			dist = normal.Length();
			if (dist != 0) {
				normal = b2Vec2(normal.x / dist, normal.y / dist);
			} else {
				normal = transform_A.p - transform_B.p;
				normal.Normalize();
			}
			//normal = -intersection_result.world_manifold.normal;
			if (intersection_result.world_manifold.normal.x != 0.0 && intersection_result.world_manifold.normal.y != 0.0) {
				//normal = -intersection_result.world_manifold.normal;
				//normal.Normalize();
			}
			return ContactResult{
				intersection_result.distance_output.distance <= margin,
				dist >= 0.0,
				dist,
				point_A,
				point_B,
				normal,
				-normal,
			};
		}
	}
	return ContactResult{
		false
	};
}

b2World *box2d::world_create(const WorldSettings *settings) {
	return memnew(b2World(b2Vec2_zero));
}

void box2d::world_destroy(b2World *world_handle) {
	memfree(world_handle);
}

size_t box2d::world_get_active_objects_count(b2World *world_handle) {
	return holder.active_objects[world_handle];
}

void box2d::world_set_active_body_callback(b2World *world_handle, ActiveBodyCallback callback) {
	holder.active_body_callbacks[world_handle] = callback;
}

void box2d::world_set_collision_filter_callback(b2World *world_handle,
		b2ContactFilter *callback) {
	world_handle->SetContactFilter(callback);
}

void box2d::world_set_contact_listener(b2World *world_handle,
		b2ContactListener *callback) {
	world_handle->SetContactListener(callback);
}

void box2d::world_step(b2World *world_handle, const SimulationSettings *settings) {
	world_handle->SetGravity(settings->gravity);
	world_handle->Step(settings->dt, settings->max_velocity_iterations, settings->max_position_iterations);
	int active_objects = 0;
	if (holder.active_body_callbacks.has(world_handle)) {
		ActiveBodyCallback callback = holder.active_body_callbacks[world_handle];
		for (b2Body *body = world_handle->GetBodyList(); body != nullptr; body = body->GetNext()) {
			if (body->GetType() == b2_kinematicBody) {
				b2BodyUserData &userData = body->GetUserData();
				userData.old_angular_velocity = body->GetAngularVelocity();
				userData.old_linear_velocity = b2Vec2_to_Vector2(body->GetLinearVelocity());
				body->SetLinearVelocity(b2Vec2_zero);
				body->SetAngularVelocity(0);
			}
			Vector2 constant_force = body->GetUserData().constant_force;
			if (constant_force != Vector2()) {
				body->ApplyForceToCenter(Vector2_to_b2Vec2(constant_force), true);
			}
			real_t constant_torque = body->GetUserData().constant_torque;
			if (constant_torque != 0.0) {
				body->ApplyTorque(constant_torque, true);
			}
			if (body->IsAwake() && body->GetUserData().collision_object->get_type() == Box2DCollisionObject2D::Type::TYPE_BODY) {
				active_objects++;
				ActiveBodyInfo info{ body, body->GetUserData() };
				callback(info);
			}
		}
	}
	holder.active_objects[world_handle] = active_objects;
}
