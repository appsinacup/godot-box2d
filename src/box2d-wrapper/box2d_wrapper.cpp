#include "box2d_wrapper.h"

using namespace box2d;

bool box2d::are_handles_equal(Handle handle1, Handle handle2) {
	return false;
}

void box2d::body_add_force(Handle world_handle, Handle body_handle, const Vector *force) {
}

void box2d::body_add_force_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *force,
		const Vector *point) {
}

void box2d::body_add_torque(Handle world_handle, Handle body_handle, Real torque) {
}

void box2d::body_apply_impulse(Handle world_handle, Handle body_handle, const Vector *impulse) {
}

void box2d::body_apply_impulse_at_point(Handle world_handle,
		Handle body_handle,
		const Vector *impulse,
		const Vector *point) {
}

void box2d::body_apply_torque_impulse(Handle world_handle, Handle body_handle, Real torque_impulse) {
}

void box2d::body_change_mode(Handle world_handle, Handle body_handle, BodyType body_type, bool wakeup) {
}

Handle box2d::body_create(Handle world_handle,
		const Vector *pos,
		Real rot,
		const UserData *user_data,
		BodyType body_type) {
	return invalid_handle();
}

void box2d::body_destroy(Handle world_handle, Handle body_handle) {
}

void box2d::body_force_sleep(Handle world_handle, Handle body_handle) {
}

Real box2d::body_get_angle(Handle world_handle, Handle body_handle) {
	return 0.0;
}

Real box2d::body_get_angular_velocity(Handle world_handle, Handle body_handle) {
	return 0.0;
}

Vector box2d::body_get_constant_force(Handle world_handle, Handle body_handle) {
	return Vector();
}

Real box2d::body_get_constant_torque(Handle world_handle, Handle body_handle) {
	return 0.0;
}

Vector box2d::body_get_linear_velocity(Handle world_handle, Handle body_handle) {
	return Vector();
}

Vector box2d::body_get_position(Handle world_handle, Handle body_handle) {
	return Vector();
}

bool box2d::body_is_ccd_enabled(Handle world_handle, Handle body_handle) {
	return false;
}

void box2d::body_reset_forces(Handle world_handle, Handle body_handle) {
}

void box2d::body_reset_torques(Handle world_handle, Handle body_handle) {
}

void box2d::body_set_angular_damping(Handle world_handle, Handle body_handle, Real angular_damping) {
}

void box2d::body_set_angular_velocity(Handle world_handle, Handle body_handle, Real vel) {
}

void box2d::body_set_can_sleep(Handle world_handle, Handle body_handle, bool can_sleep) {
}

void box2d::body_set_ccd_enabled(Handle world_handle, Handle body_handle, bool enable) {
}

void box2d::body_set_gravity_scale(Handle world_handle,
		Handle body_handle,
		Real gravity_scale,
		bool wake_up) {
}

void box2d::body_set_linear_damping(Handle world_handle, Handle body_handle, Real linear_damping) {
}

void box2d::body_set_linear_velocity(Handle world_handle, Handle body_handle, const Vector *vel) {
}

void box2d::body_set_mass_properties(Handle world_handle,
		Handle body_handle,
		Real mass,
		Real inertia,
		const Vector *local_com,
		bool wake_up,
		bool force_update) {
}

void box2d::body_set_transform(Handle world_handle,
		Handle body_handle,
		const Vector *pos,
		Real rot,
		bool wake_up) {
}

void box2d::body_update_material(Handle world_handle, Handle body_handle, const Material *mat) {
}

void box2d::body_wake_up(Handle world_handle, Handle body_handle, bool strong) {
}

Handle box2d::collider_create_sensor(Handle world_handle,
		Handle shape_handle,
		Handle body_handle,
		const UserData *user_data) {
	return invalid_handle();
}

Handle box2d::collider_create_solid(Handle world_handle,
		Handle shape_handle,
		const Material *mat,
		Handle body_handle,
		const UserData *user_data) {
	return invalid_handle();
}

void box2d::collider_destroy(Handle world_handle, Handle handle) {
}

Real box2d::collider_get_angle(Handle world_handle, Handle handle) {
	return 0.0;
}

Vector box2d::collider_get_position(Handle world_handle, Handle handle) {
	return Vector();
}

void box2d::collider_set_collision_events_enabled(Handle world_handle, Handle handle, bool enable) {
}

void box2d::collider_set_contact_force_events_enabled(Handle world_handle, Handle handle, bool enable) {
}

void box2d::collider_set_transform(Handle world_handle, Handle handle, ShapeInfo shape_info) {
}

Material box2d::default_material() {
	return Material{};
}

QueryExcludedInfo box2d::default_query_excluded_info() {
	return QueryExcludedInfo{};
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
	return Handle{};
}

UserData box2d::invalid_user_data() {
	return UserData{};
}

bool box2d::is_handle_valid(Handle handle) {
	return false;
}

bool box2d::is_user_data_valid(UserData user_data) {
	return false;
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
	return invalid_handle();
}

Handle box2d::shape_create_capsule(Real half_height, Real radius) {
	return invalid_handle();
}

Handle box2d::shape_create_circle(Real radius) {
	return invalid_handle();
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
}

ContactResult box2d::shapes_contact(Handle world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		Real margin) {
	return ContactResult{};
}

Handle box2d::world_create(const WorldSettings *settings) {
	return invalid_handle();
}

void box2d::world_destroy(Handle world_handle) {
}

size_t box2d::world_get_active_objects_count(Handle world_handle) {
	return 0;
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
}
