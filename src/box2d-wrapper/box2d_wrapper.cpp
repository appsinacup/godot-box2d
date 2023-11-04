#include "box2d_wrapper.h"
#include "box2d_holder.h"
#include <box2d/box2d.h>
#include <godot_cpp/templates/hash_set.hpp>

using namespace box2d;

enum class ShapeType {
	Circle = 0,
	Segment,
	Polygon,
};

Box2DHolder holder;

Handle new_shape_handle(ShapeType type) {
	return Handle{ -1, rand(), -1, static_cast<uint16_t>(type) };
}

bool box2d::are_handles_equal(Handle handle1, Handle handle2) {
	return handle1.object_index == handle2.object_index &&
			handle1.revision == handle2.revision &&
			handle1.world == handle2.world &&
			handle1.world_index == handle2.world_index;
}

void box2d::body_add_force(Handle world_handle, Handle body_handle, const Vector *force) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->ApplyForceToCenter(vector_to_b2_vec(*force), true);
}

void box2d::body_add_force_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *force,
		const Vector *point) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->ApplyForce(vector_to_b2_vec(*force), vector_to_b2_vec(*point), true);
}

void box2d::body_add_torque(Handle world_handle, Handle body_handle, Real torque) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->ApplyTorque(torque, true);
}

void box2d::body_apply_impulse(Handle world_handle, Handle body_handle, const Vector *impulse) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->ApplyLinearImpulseToCenter(vector_to_b2_vec(*impulse), true);
}

void box2d::body_apply_impulse_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *impulse,
		const Vector *point) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->ApplyLinearImpulse(vector_to_b2_vec(*impulse), vector_to_b2_vec(*point), true);
}

void box2d::body_apply_torque_impulse(Handle world_handle, Handle body_handle, Real torque_impulse) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->ApplyAngularImpulse(torque_impulse, true);
}

void box2d::body_change_mode(Handle world_handle, Handle body_handle, BodyType body_type, bool wakeup) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetType(body_type_to_b2_body_type(body_type));
}

Handle box2d::body_create(Handle world_handle,
		const Vector *pos,
		Real rot,
		const UserData *user_data,
		BodyType body_type) {
	b2BodyDef body_def;
	body_def.position = b2Vec2{ pos->x, pos->y };
	body_def.angle = rot;
	body_def.userData = (void *)user_data;
	body_def.type = body_type_to_b2_body_type(body_type);
	b2World *world = holder.handle_to_world(world_handle);
	b2Body *body = world->CreateBody(&body_def);
	return holder.body_to_handle(body);
}

void box2d::body_destroy(Handle world_handle, Handle body_handle) {
	b2World *world = holder.handle_to_world(world_handle);
	b2Body *body = holder.handle_to_body(body_handle);
	world->DestroyBody(body);
}

void box2d::body_force_sleep(Handle world_handle, Handle body_handle) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetAwake(false);
}

Real box2d::body_get_angle(Handle world_handle, Handle body_handle) {
	b2Body *body = holder.handle_to_body(body_handle);
	return body->GetAngle();
}

Real box2d::body_get_angular_velocity(Handle world_handle, Handle body_handle) {
	b2Body *body = holder.handle_to_body(body_handle);
	return body->GetAngularVelocity();
}

Vector box2d::body_get_constant_force(Handle world_handle, Handle body_handle) {
	return Vector();
}

Real box2d::body_get_constant_torque(Handle world_handle, Handle body_handle) {
	return 0.0;
}

Vector box2d::body_get_linear_velocity(Handle world_handle, Handle body_handle) {
	b2Body *body = holder.handle_to_body(body_handle);
	return b2_vec_to_vector(body->GetLinearVelocity());
}

Vector box2d::body_get_position(Handle world_handle, Handle body_handle) {
	b2Body *body = holder.handle_to_body(body_handle);
	return b2_vec_to_vector(body->GetPosition());
}

bool box2d::body_is_ccd_enabled(Handle world_handle, Handle body_handle) {
	b2Body *body = holder.handle_to_body(body_handle);
	return body->IsBullet();
}

void box2d::body_reset_forces(Handle world_handle, Handle body_handle) {
}

void box2d::body_reset_torques(Handle world_handle, Handle body_handle) {
}

void box2d::body_set_angular_damping(Handle world_handle, Handle body_handle, Real angular_damping) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetAngularDamping(angular_damping);
}

void box2d::body_set_angular_velocity(Handle world_handle, Handle body_handle, Real vel) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetAngularVelocity(vel);
}

void box2d::body_set_can_sleep(Handle world_handle, Handle body_handle, bool can_sleep) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetSleepingAllowed(can_sleep);
}

void box2d::body_set_ccd_enabled(Handle world_handle, Handle body_handle, bool enable) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetBullet(enable);
}

void box2d::body_set_gravity_scale(Handle world_handle,
		Handle body_handle,
		Real gravity_scale,
		bool wake_up) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetGravityScale(gravity_scale);
}

void box2d::body_set_linear_damping(Handle world_handle, Handle body_handle, Real linear_damping) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetLinearDamping(linear_damping);
}

void box2d::body_set_linear_velocity(Handle world_handle, Handle body_handle, const Vector *vel) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetLinearVelocity(vector_to_b2_vec(*vel));
}

void box2d::body_set_mass_properties(Handle world_handle,
		Handle body_handle,
		Real mass,
		Real inertia,
		const Vector *local_com,
		bool wake_up,
		bool force_update) {
	b2MassData mass_data{
		mass,
		vector_to_b2_vec(*local_com),
		inertia
	};
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetMassData(&mass_data);
}

void box2d::body_set_transform(Handle world_handle,
		Handle body_handle,
		const Vector *pos,
		Real rot,
		bool wake_up) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetTransform(vector_to_b2_vec(*pos), rot);
	// TODO kinematic
}

void box2d::body_update_material(Handle world_handle, Handle body_handle, const Material *mat) {
	b2Body *body = holder.handle_to_body(body_handle);
	for (b2Fixture *fixture = body->GetFixtureList(); fixture = fixture->GetNext()) {
		fixture->SetFriction(mat->friction);
		fixture->SetRestitution(mat->restitution);
	}
}

void box2d::body_wake_up(Handle world_handle, Handle body_handle, bool strong) {
	b2Body *body = holder.handle_to_body(body_handle);
	body->SetAwake(true);
}

Handle box2d::collider_create_sensor(Handle world_handle,
		Handle shape_handle,
		Handle body_handle,
		const UserData *user_data) {
	b2Body *body = holder.handle_to_body(body_handle);
	b2Shape *shape = holder.handle_to_shape(shape_handle);
	b2FixtureDef fixture_def;
	fixture_def.shape = shape;
	fixture_def.density = 1.0f;
	fixture_def.isSensor = true;
	fixture_def.userData = user_data;
	b2Fixture *fixture = body->CreateFixture(&fixture_def);
	return holder.collider_to_handle(fixture);
}

Handle box2d::collider_create_solid(Handle world_handle,
		Handle shape_handle,
		const Material *mat,
		Handle body_handle,
		const UserData *user_data) {
	b2Body *body = holder.handle_to_body(body_handle);
	b2Shape *shape = holder.handle_to_shape(shape_handle);
	b2FixtureDef fixture_def;
	fixture_def.shape = shape;
	fixture_def.density = 1.0f;
	fixture_def.isSensor = false;
	fixture_def.userData = user_data;
	b2Fixture *fixture = body->CreateFixture(&fixture_def);
	return holder.collider_to_handle(fixture);
}

void box2d::collider_destroy(Handle world_handle, Handle handle) {
	b2Fixture *fixture = holder.handle_to_collider(handle);
	fixture->GetBody()->DestroyFixture(fixture);
}

Real box2d::collider_get_angle(Handle world_handle, Handle handle) {
	b2Transform *transform = holder.handle_to_collider_transform(handle);
	return transform->q.GetAngle();
}

Vector box2d::collider_get_position(Handle world_handle, Handle handle) {
	b2Transform *transform = holder.handle_to_collider_transform(handle);
	return b2_vec_to_vector(transform->p);
}

void box2d::collider_set_collision_events_enabled(Handle world_handle, Handle handle, bool enable) {
}

void box2d::collider_set_contact_force_events_enabled(Handle world_handle, Handle handle, bool enable) {
}

void box2d::collider_set_transform(Handle world_handle, Handle handle, ShapeInfo shape_info) {
	b2Transform *transform = holder.handle_to_collider_transform(handle);
	return b2_vec_to_vector(transform->p);
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

size_t box2d::intersect_aabb(Handle world_handle,
		const Vector *aabb_min,
		const Vector *aabb_max,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return 0;
}

size_t box2d::intersect_point(Handle world_handle,
		const Vector *position,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return 0;
}

bool box2d::intersect_ray(Handle world_handle,
		const Vector *from,
		const Vector *dir,
		Real length,
		bool collide_with_body,
		bool collide_with_area,
		bool hit_from_inside,
		RayHitInfo *hit_info,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return false;
}

size_t box2d::intersect_shape(Handle world_handle,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return 0;
}

Handle box2d::invalid_handle() {
	return Handle{ -1, -1, -1, 0 };
}

UserData box2d::invalid_user_data() {
	return UserData{ uint64_t(-1), uint64_t(-1) };
}

bool box2d::is_handle_valid(Handle handle) {
	return handle.object_index == -1 || handle.world == -1 || handle.world_index == -1;
}

bool box2d::is_user_data_valid(UserData user_data) {
	return user_data.part1 == uint64_t(-1) || user_data.part2 == uint64_t(-1);
}

void box2d::joint_change_revolute_params(Handle world_handle,
		Handle joint_handle,
		Real angular_limit_lower,
		Real angular_limit_upper,
		bool angular_limit_enabled,
		Real motor_target_velocity,
		bool motor_enabled) {
}

Handle box2d::joint_create_prismatic(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *axis,
		const Vector *anchor_1,
		const Vector *anchor_2,
		const Vector *limits) {
	return invalid_handle();
}

Handle box2d::joint_create_revolute(Handle world_handle,
		Handle body_handle_1,
		Handle body_handle_2,
		const Vector *anchor_1,
		const Vector *anchor_2,
		Real angular_limit_lower,
		Real angular_limit_upper,
		bool angular_limit_enabled,
		Real motor_target_velocity,
		bool motor_enabled) {
	return invalid_handle();
}

void box2d::joint_destroy(Handle world_handle, Handle joint_handle) {
}

ShapeCastResult box2d::shape_casting(Handle world_handle,
		const Vector *motion,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info) {
	return ShapeCastResult{};
}

ShapeCastResult box2d::shape_collide(const Vector *motion1,
		ShapeInfo shape_info1,
		const Vector *motion2,
		ShapeInfo shape_info2) {
	return ShapeCastResult{};
}

Handle box2d::shape_create_box(const Vector *size) {
	b2PolygonShape *shape = memnew(b2PolygonShape);
	shape->SetAsBox(size->x, size->y);
	return holder.shape_to_handle(shape);
}

Handle box2d::shape_create_capsule(Real half_height, Real radius) {
	return invalid_handle();
}

Handle box2d::shape_create_circle(Real radius) {
	b2CircleShape *shape = memnew(b2CircleShape);
	shape->m_radius = radius;
	return holder.shape_to_handle(shape);
}

Handle box2d::shape_create_convave_polyline(const Vector *points, size_t point_count) {
	return invalid_handle();
}

Handle box2d::shape_create_convex_polyline(const Vector *points, size_t point_count) {
	return invalid_handle();
}

Handle box2d::shape_create_halfspace(const Vector *normal) {
	return invalid_handle();
}

void box2d::shape_destroy(Handle shape_handle) {
	b2Shape *shape = holder.handle_to_shape(shape_handle);
	memfree(shape);
}

ContactResult box2d::shapes_contact(Handle world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		Real margin) {
	return ContactResult{};
}

Handle box2d::world_create(const WorldSettings *settings) {
	b2World *world = memnew(b2World(b2Vec2_zero));
	return holder.world_to_handle(world);
}

void box2d::world_destroy(Handle world_handle) {
	b2World *world = holder.handle_to_world(world_handle);
	memfree(world);
}

size_t box2d::world_get_active_objects_count(Handle world_handle) {
	return 0.0;
}

void box2d::world_set_active_body_callback(Handle world_handle, ActiveBodyCallback callback) {
}

void box2d::world_set_body_collision_filter_callback(Handle world_handle,
		CollisionFilterCallback callback) {
}

void box2d::world_set_collision_event_callback(Handle world_handle, CollisionEventCallback callback) {
}

void box2d::world_set_contact_force_event_callback(Handle world_handle,
		ContactForceEventCallback callback) {
}

void box2d::world_set_contact_point_callback(Handle world_handle, ContactPointCallback callback) {
}

void box2d::world_set_modify_contacts_callback(Handle world_handle,
		CollisionModifyContactsCallback callback) {
}

void box2d::world_set_sensor_collision_filter_callback(Handle world_handle,
		CollisionFilterCallback callback) {
}

void box2d::world_step(Handle world_handle, const SimulationSettings *settings) {
	b2World *world = holder.handle_to_world(world_handle);
	world->SetGravity(vector_to_b2_vec(settings->gravity));
	world->Step(settings->dt, settings->max_velocity_iterations, 3);
}
