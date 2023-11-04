#include "box2d_wrapper.h"
#include "box2d_helper.h"
#include "../b2_user_settings.h"
#include <box2d/box2d.h>
#include <godot_cpp/templates/hash_set.hpp>

using namespace box2d;

enum class ShapeType {
	Circle = 0,
	Segment,
	Polygon,
};

Box2DHolder holder;

bool box2d::are_handles_equal(b2World* handle1, b2World* handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(b2Body* handle1, b2Body* handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(b2Fixture* handle1, b2Fixture* handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(b2Shape* handle1, b2Shape* handle2) {
	return handle1 == handle2;
}
bool box2d::are_handles_equal(b2Joint* handle1, b2Joint* handle2) {
	return handle1 == handle2;
}

void box2d::body_add_force(b2World* world_handle, b2Body* body_handle, const b2Vec2 force) {
	body_handle->ApplyForceToCenter(force, true);
}

void box2d::body_add_force_at_point(b2World* world_handle,
		b2Body* body_handle,
		const b2Vec2 force,
		const b2Vec2 point) {
	body_handle->ApplyForce(force, point, true);
}

void box2d::body_add_torque(b2World* world_handle, b2Body* body_handle, real_t torque) {
	body_handle->ApplyTorque(torque, true);
}

void box2d::body_apply_impulse(b2World* world_handle, b2Body* body_handle, const b2Vec2 impulse) {
	body_handle->ApplyLinearImpulseToCenter(impulse, true);
}

void box2d::body_apply_impulse_at_point(b2World* world_handle,
		b2Body* body_handle,
		const b2Vec2 impulse,
		const b2Vec2 point) {
	body_handle->ApplyLinearImpulse(impulse, point, true);
}

void box2d::body_apply_torque_impulse(b2World* world_handle, b2Body* body_handle, real_t torque_impulse) {
	body_handle->ApplyAngularImpulse(torque_impulse, true);
}

void box2d::body_change_mode(b2World* world_handle, b2Body* body_handle, b2BodyType body_type, bool wakeup) {
	body_handle->SetType(body_type);
}

b2Body* box2d::body_create(b2World* world_handle,
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

void box2d::body_destroy(b2World* world_handle, b2Body* body_handle) {
	world_handle->DestroyBody(body_handle);
}

void box2d::body_force_sleep(b2World* world_handle, b2Body* body_handle) {
	body_handle->SetAwake(false);
}

real_t box2d::body_get_angle(b2World* world_handle, b2Body* body_handle) {
	return body_handle->GetAngle();
}

real_t box2d::body_get_angular_velocity(b2World* world_handle, b2Body* body_handle) {
	return body_handle->GetAngularVelocity();
}

b2Vec2 box2d::body_get_constant_force(b2World* world_handle, b2Body* body_handle) {
	return b2Vec2();
}

real_t box2d::body_get_constant_torque(b2World* world_handle, b2Body* body_handle) {
	return 0.0;
}

b2Vec2 box2d::body_get_linear_velocity(b2World* world_handle, b2Body* body_handle) {
	return body_handle->GetLinearVelocity();
}

b2Vec2 box2d::body_get_position(b2World* world_handle, b2Body* body_handle) {
	return body_handle->GetPosition();
}

bool box2d::body_is_ccd_enabled(b2World* world_handle, b2Body* body_handle) {
	return body_handle->IsBullet();
}

void box2d::body_reset_forces(b2World* world_handle, b2Body* body_handle) {
}

void box2d::body_reset_torques(b2World* world_handle, b2Body* body_handle) {
}

void box2d::body_set_angular_damping(b2World* world_handle, b2Body* body_handle, real_t angular_damping) {
	body_handle->SetAngularDamping(angular_damping);
}

void box2d::body_set_angular_velocity(b2World* world_handle, b2Body* body_handle, real_t vel) {
	body_handle->SetAngularVelocity(vel);
}

void box2d::body_set_can_sleep(b2World* world_handle, b2Body* body_handle, bool can_sleep) {
	body_handle->SetSleepingAllowed(can_sleep);
}

void box2d::body_set_ccd_enabled(b2World* world_handle, b2Body* body_handle, bool enable) {
	body_handle->SetBullet(enable);
}

void box2d::body_set_gravity_scale(b2World* world_handle,
		b2Body* body_handle,
		real_t gravity_scale,
		bool wake_up) {
	body_handle->SetGravityScale(gravity_scale);
}

void box2d::body_set_linear_damping(b2World* world_handle, b2Body* body_handle, real_t linear_damping) {
	body_handle->SetLinearDamping(linear_damping);
}

void box2d::body_set_linear_velocity(b2World* world_handle, b2Body* body_handle, const b2Vec2 vel) {
	body_handle->SetLinearVelocity(vel);
}

void box2d::body_set_mass_properties(b2World* world_handle,
		b2Body* body_handle,
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

void box2d::body_set_transform(b2World* world_handle,
		b2Body* body_handle,
		const b2Vec2 pos,
		real_t rot,
		bool wake_up) {
	body_handle->SetTransform(pos, rot);
	// TODO kinematic
}

void box2d::body_update_material(b2World* world_handle, b2Body* body_handle, const Material *mat) {
	for (b2Fixture *fixture = body_handle->GetFixtureList(); fixture != nullptr; fixture = fixture->GetNext()) {
		fixture->SetFriction(mat->friction);
		fixture->SetRestitution(mat->restitution);
	}
}

void box2d::body_wake_up(b2World* world_handle, b2Body* body_handle, bool strong) {
	body_handle->SetAwake(true);
}

b2Fixture* box2d::collider_create_sensor(b2World* world_handle,
		b2Shape* shape_handle,
		b2Body* body_handle,
		const b2FixtureUserData user_data) {
	b2FixtureDef fixture_def;
	fixture_def.shape = shape_handle;
	fixture_def.density = 1.0f;
	fixture_def.isSensor = true;
	fixture_def.userData = user_data;
	return body_handle->CreateFixture(&fixture_def);
}

b2Fixture* box2d::collider_create_solid(b2World* world_handle,
		b2Shape* shape_handle,
		const Material *mat,
		b2Body* body_handle,
		const b2FixtureUserData user_data) {
	b2FixtureDef fixture_def;
	fixture_def.shape = shape_handle;
	fixture_def.density = 1.0f;
	fixture_def.isSensor = false;
	fixture_def.userData = user_data;
	return body_handle->CreateFixture(&fixture_def);
}

void box2d::collider_destroy(b2World* world_handle, b2Fixture* handle) {
	handle->GetBody()->DestroyFixture(handle);
}

real_t box2d::collider_get_angle(b2World* world_handle, b2Fixture* handle) {
	b2Transform *transform = holder.handle_to_fixture_transform(handle);
	return transform->q.GetAngle();
}

b2Vec2 box2d::collider_get_position(b2World* world_handle, b2Fixture* handle) {
	b2Transform *transform = holder.handle_to_fixture_transform(handle);
	return transform->p;
}

void box2d::collider_set_collision_events_enabled(b2World* world_handle, b2Fixture* handle, bool enable) {
}

void box2d::collider_set_contact_force_events_enabled(b2World* world_handle, b2Fixture* handle, bool enable) {
}

void box2d::collider_set_transform(b2World* world_handle, b2Fixture* handle, ShapeInfo shape_info) {
	b2Transform *transform = holder.handle_to_fixture_transform(handle);
	//return transform->p);
	// TODO
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

size_t box2d::intersect_aabb(b2World* world_handle,
		const b2Vec2 aabb_min,
		const b2Vec2 aabb_max,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return 0;
}

size_t box2d::intersect_point(b2World* world_handle,
		const b2Vec2 position,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return 0;
}

bool box2d::intersect_ray(b2World* world_handle,
		const b2Vec2 from,
		const b2Vec2 dir,
		real_t length,
		bool collide_with_body,
		bool collide_with_area,
		bool hit_from_inside,
		RayHitInfo *hit_info,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return false;
}

size_t box2d::intersect_shape(b2World* world_handle,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return 0;
}

b2World* box2d::invalid_world_handle() {
	return nullptr;
}
b2Fixture* box2d::invalid_fixture_handle() {
	return nullptr;
}
b2Body* box2d::invalid_body_handle() {
	return nullptr;
}
b2Shape* box2d::invalid_shape_handle() {
	return nullptr;
}
b2Joint* box2d::invalid_joint_handle() {
	return nullptr;
}

b2FixtureUserData box2d::invalid_fixture_user_data() {
	return b2FixtureUserData{};
}
b2BodyUserData box2d::invalid_body_user_data() {
	return b2BodyUserData{};
}
bool box2d::is_user_data_valid(b2FixtureUserData user_data) {
	return user_data.shape != nullptr;
}
bool box2d::is_user_data_valid(b2BodyUserData user_data) {
	return user_data.collision_object != nullptr;
}

bool box2d::is_handle_valid(b2Fixture *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(b2World *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(b2Shape *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(b2Body *handle) {
	return handle != nullptr;
}
bool box2d::is_handle_valid(b2Joint *handle) {
	return handle != nullptr;
}

void box2d::joint_change_revolute_params(b2World* world_handle,
		b2Joint* joint_handle,
		real_t angular_limit_lower,
		real_t angular_limit_upper,
		bool angular_limit_enabled,
		real_t motor_target_velocity,
		bool motor_enabled) {
}

b2Joint* box2d::joint_create_prismatic(b2World* world_handle,
		b2Body* body_handle_1,
		b2Body* body_handle_2,
		const b2Vec2 axis,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		const b2Vec2 limits) {
	return nullptr;
}

b2Joint* box2d::joint_create_revolute(b2World* world_handle,
		b2Body* body_handle_1,
		b2Body* body_handle_2,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		real_t angular_limit_lower,
		real_t angular_limit_upper,
		bool angular_limit_enabled,
		real_t motor_target_velocity,
		bool motor_enabled) {
	return nullptr;
}

void box2d::joint_destroy(b2World* world_handle, b2Joint* joint_handle) {
	world_handle->DestroyJoint(joint_handle);
}

ShapeCastResult box2d::shape_casting(b2World* world_handle,
		const b2Vec2 motion,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return ShapeCastResult{};
}

ShapeCastResult box2d::shape_collide(const b2Vec2 motion1,
		ShapeInfo shape_info1,
		const b2Vec2 motion2,
		ShapeInfo shape_info2) {
	return ShapeCastResult{};
}

b2Shape* box2d::shape_create_box(const b2Vec2 size) {
	b2PolygonShape *shape = memnew(b2PolygonShape);
	shape->SetAsBox(size.x, size.y);
	return shape
}

b2Shape* box2d::shape_create_capsule(real_t half_height, real_t radius) {
	return nullptr;
}

b2Shape* box2d::shape_create_circle(real_t radius) {
	b2CircleShape *shape = memnew(b2CircleShape);
	shape->m_radius = radius;
	return shape;
}

b2Shape* box2d::shape_create_convave_polyline(const b2Vec2 *points, size_t point_count) {
	return nullptr;
}

b2Shape* box2d::shape_create_convex_polyline(const b2Vec2 *points, size_t point_count) {
	return nullptr;
}

b2Shape* box2d::shape_create_halfspace(const b2Vec2 normal) {
	return nullptr;
}

void box2d::shape_destroy(b2Shape* shape_handle) {
	memfree(shape_handle);
}

ContactResult box2d::shapes_contact(b2World* world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		real_t margin) {
	return ContactResult{};
}

b2World* box2d::world_create(const WorldSettings *settings) {
	return memnew(b2World(b2Vec2_zero));
}

void box2d::world_destroy(b2World* world_handle) {
	memfree(world_handle);
}

size_t box2d::world_get_active_objects_count(b2World* world_handle) {
	return 0.0;
}

void box2d::world_set_active_body_callback(b2World* world_handle, ActiveBodyCallback callback) {
}

void box2d::world_set_body_collision_filter_callback(b2World* world_handle,
		CollisionFilterCallback callback) {
}

void box2d::world_set_collision_event_callback(b2World* world_handle, CollisionEventCallback callback) {
}

void box2d::world_set_contact_force_event_callback(b2World* world_handle,
		ContactForceEventCallback callback) {
}

void box2d::world_set_contact_point_callback(b2World* world_handle, ContactPointCallback callback) {
}

void box2d::world_set_modify_contacts_callback(b2World* world_handle,
		CollisionModifyContactsCallback callback) {
}

void box2d::world_set_sensor_collision_filter_callback(b2World* world_handle,
		CollisionFilterCallback callback) {
}

void box2d::world_step(b2World* world_handle, const SimulationSettings *settings) {
	world_handle->SetGravity(settings->gravity);
	world_handle->Step(settings->dt, settings->max_velocity_iterations, 3);
}
