#ifndef BOX2D_WRAPPER_H
#define BOX2D_WRAPPER_H

#include "../b2_user_settings.h"

#include <box2d/box2d.h>
#include <box2d/distance.h>
#include <stdio.h>
#include <cstdint>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/variant/transform2d.hpp>

class b2FixtureUserData;
class b2BodyUserData;

namespace box2d {

struct Material {
	real_t friction;
	real_t restitution;
};

struct ShapeHandle {
	b2ShapeId **handles;
	int count;
};

struct FixtureHandle {
	b2Fixture **handles;
	int count;
};

struct ShapeInfo {
	ShapeHandle handle;
	godot::Transform2D body_transform;
	godot::Transform2D transform;
};

struct QueryExcludedInfo {
	uint32_t query_collision_layer_mask;
	uint64_t query_canvas_instance_id;
	b2Fixture **query_exclude;
	uint32_t query_exclude_size;
	int64_t query_exclude_body;
};

struct WorldSettings {
	real_t sleep_linear_threshold;
	real_t sleep_angular_threshold;
	real_t sleep_time_until_sleep;
	real_t solver_prediction_distance;
};

struct PointHitInfo {
	b2Fixture *collider;
	b2FixtureUserData user_data;
};

using QueryHandleExcludedCallback = bool (*)(b2WorldId world_handle,
		b2Fixture *collider_handle,
		b2FixtureUserData user_data,
		const QueryExcludedInfo *handle_excluded_info);

struct RayHitInfo {
	b2Vec2 position;
	b2Vec2 normal;
	b2Fixture *collider;
	b2FixtureUserData user_data;
};

struct ShapeCollideResult {
	bool collided;
	b2Vec2 witness1;
	b2Vec2 witness2;
};

struct ShapeCastResult {
	bool collided;
	real_t toi;
	b2Vec2 witness1;
	b2Vec2 witness2;
	b2Vec2 normal1;
	b2Vec2 normal2;
	b2Fixture *collider;
	b2FixtureUserData user_data;
};

struct ContactResult {
	bool collided;
	bool within_margin;
	real_t distance;
	b2Vec2 point1;
	b2Vec2 point2;
	b2Vec2 normal1;
	b2Vec2 normal2;
};

struct ActiveBodyInfo {
	b2BodyId body_handle;
	b2BodyUserData body_user_data;
};

using ActiveBodyCallback = void (*)(const ActiveBodyInfo &active_body_info);

struct CollisionFilterInfo {
	b2FixtureUserData user_data1;
	b2FixtureUserData user_data2;
	bool is_valid;
};

using CollisionFilterCallback = bool (*)(b2WorldId world_handle, const CollisionFilterInfo *filter_info);

struct CollisionEventInfo {
	b2Fixture *collider1;
	b2Fixture *collider2;
	b2FixtureUserData user_data1;
	b2FixtureUserData user_data2;
	bool is_sensor;
	bool is_started;
	bool is_removed;
	bool is_valid;
};

using CollisionEventCallback = void (*)(b2WorldId world_handle, const CollisionEventInfo *event_info);

struct ContactForceEventInfo {
	b2Fixture *collider1;
	b2Fixture *collider2;
	b2FixtureUserData user_data1;
	b2FixtureUserData user_data2;
	bool is_valid;
};

using ContactForceEventCallback = bool (*)(b2WorldId world_handle,
		const ContactForceEventInfo *event_info);

struct ContactPointInfo {
	b2Vec2 local_pos_1;
	b2Vec2 local_pos_2;
	b2Vec2 velocity_pos_1;
	b2Vec2 velocity_pos_2;
	b2Vec2 normal_1;
	b2Vec2 normal_2;
	real_t distance_1;
	real_t distance_2;
	real_t impulse_1;
	real_t tangent_impulse_1;
	real_t impulse_2;
	real_t tangent_impulse_2;
};

using ContactPointCallback = bool (*)(b2WorldId world_handle,
		const ContactPointInfo *contact_info,
		const ContactForceEventInfo *event_info);

struct OneWayDirection {
	bool body1;
	bool body2;
	real_t body1_margin;
	real_t body2_margin;
	real_t last_timestep;
};

using CollisionModifyContactsCallback = OneWayDirection (*)(b2WorldId world_handle,
		const CollisionFilterInfo *filter_info);

struct SimulationSettings {
	real_t dt;
	size_t max_velocity_iterations;
	size_t max_position_iterations;
	b2Vec2 gravity;
};

b2Vec2 Vector2_to_b2Vec2(godot::Vector2 vec);
godot::Vector2 b2Vec2_to_Vector2(b2Vec2 vec);

bool are_handles_equal(b2WorldId handle1, b2WorldId handle2);
bool are_handles_equal(b2BodyId handle1, b2BodyId handle2);
bool are_handles_equal(FixtureHandle handle1, FixtureHandle handle2);
bool are_handles_equal(ShapeHandle handle1, ShapeHandle handle2);
bool are_handles_equal(b2JointId handle1, b2JointId handle2);
bool are_handles_equal(b2Fixture *handle1, b2Fixture *handle2);

void body_add_force(b2WorldId world_handle, b2BodyId body_handle, const b2Vec2 force);

void body_add_torque(b2WorldId world_handle, b2BodyId body_handle, real_t torque);

void body_apply_impulse(b2WorldId world_handle, b2BodyId body_handle, const b2Vec2 impulse);

void body_apply_impulse_at_point(b2WorldId world_handle,
		b2BodyId body_handle,
		const b2Vec2 impulse,
		const b2Vec2 point);

void body_apply_torque_impulse(b2World *world_handle, b2Body *body_handle, real_t torque_impulse);

void body_change_mode(b2World *world_handle, b2Body *body_handle, b2BodyType body_type, bool wakeup, bool fixed_rotation);

b2Body *body_create(b2World *world_handle,
		const b2Vec2 pos,
		real_t rot,
		const b2BodyUserData &user_data,
		b2BodyType body_type);

void body_destroy(b2World *world_handle, b2Body *body_handle);

void body_force_sleep(b2World *world_handle, b2Body *body_handle);

real_t body_get_angle(b2World *world_handle, b2Body *body_handle);

real_t body_get_angular_velocity(b2World *world_handle, b2Body *body_handle);

b2Vec2 body_get_constant_force(b2World *world_handle, b2Body *body_handle);

real_t body_get_constant_torque(b2World *world_handle, b2Body *body_handle);

b2Vec2 body_get_linear_velocity(b2World *world_handle, b2Body *body_handle);

b2Vec2 body_get_position(b2World *world_handle, b2Body *body_handle);

bool body_is_ccd_enabled(b2World *world_handle, b2Body *body_handle);

void body_reset_forces(b2World *world_handle, b2Body *body_handle);

void body_reset_torques(b2World *world_handle, b2Body *body_handle);

void body_set_angular_damping(b2World *world_handle, b2Body *body_handle, real_t angular_damping);

void body_set_angular_velocity(b2World *world_handle, b2Body *body_handle, real_t vel);

void body_set_can_sleep(b2World *world_handle, b2Body *body_handle, bool can_sleep);

void body_set_ccd_enabled(b2World *world_handle, b2Body *body_handle, bool enable);

void body_set_gravity_scale(b2World *world_handle,
		b2Body *body_handle,
		real_t gravity_scale,
		bool wake_up);

void body_set_linear_damping(b2World *world_handle, b2Body *body_handle, real_t linear_damping);

void body_set_linear_velocity(b2World *world_handle, b2Body *body_handle, const b2Vec2 vel);

void body_set_mass_properties(b2World *world_handle,
		b2Body *body_handle,
		real_t mass,
		real_t inertia,
		const b2Vec2 local_com,
		bool wake_up,
		bool force_update);

void body_set_transform(b2World *world_handle,
		b2Body *body_handle,
		const b2Vec2 pos,
		real_t rot,
		bool wake_up,
		real_t step);

void body_update_material(b2World *world_handle, b2Body *body_handle, const Material *mat);

void body_wake_up(b2World *world_handle, b2Body *body_handle, bool strong);

FixtureHandle collider_create_sensor(b2World *world_handle,
		ShapeHandle shape_handle,
		b2Body *body_handle,
		b2FixtureUserData user_data);

FixtureHandle collider_create_solid(b2World *world_handle,
		ShapeHandle shape_handle,
		const Material *mat,
		b2Body *body_handle,
		b2FixtureUserData user_data);

void collider_destroy(b2World *world_handle, FixtureHandle handle);

void collider_set_transform(b2World *world_handle, FixtureHandle handle, ShapeInfo shape_info);

godot::Transform2D collider_get_transform(b2World *world_handle, FixtureHandle handle);
godot::Transform2D collider_get_transform(b2World *world_handle, b2Fixture *handle);

Material default_material();

QueryExcludedInfo default_query_excluded_info();

WorldSettings default_world_settings();

size_t intersect_aabb(b2World *world_handle,
		const b2Vec2 aabb_min,
		const b2Vec2 aabb_max,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

size_t intersect_point(b2World *world_handle,
		const b2Vec2 position,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

bool intersect_ray(b2World *world_handle,
		const b2Vec2 from,
		const b2Vec2 dir,
		real_t length,
		bool collide_with_body,
		bool collide_with_area,
		bool hit_from_inside,
		RayHitInfo *hit_info,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info);

size_t intersect_shape(b2World *world_handle,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		PointHitInfo *hit_info_array,
		size_t hit_info_length,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info,
		double margin);

b2WorldId invalid_world_handle();
FixtureHandle invalid_fixture_handle();
b2BodyId invalid_body_handle();
ShapeHandle invalid_shape_handle();
b2JointId invalid_joint_handle();

b2FixtureUserData invalid_fixture_user_data();
b2BodyUserData invalid_body_user_data();

bool is_handle_valid(FixtureHandle handle);
bool is_handle_valid(b2WorldId handle);
bool is_handle_valid(ShapeHandle handle);
bool is_handle_valid(b2BodyId handle);
bool is_handle_valid(b2JointId handle);
bool is_handle_valid(b2Fixture *handle);

bool is_user_data_valid(b2FixtureUserData user_data);
bool is_user_data_valid(b2BodyUserData user_data);

void joint_set_disable_collision(b2JointId joint_handle,
		bool disable_collision);

void joint_change_revolute_params(b2WorldId world_handle,
		b2JointId joint_handle,
		real_t angular_limit_lower,
		real_t angular_limit_upper,
		bool angular_limit_enabled,
		real_t motor_target_velocity,
		bool motor_enabled);

b2JointId joint_create_prismatic(b2WorldId world_handle,
		b2BodyId body_handle_1,
		b2BodyId body_handle_2,
		const b2Vec2 axis,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		const b2Vec2 limits,
		bool disable_collision);

b2JointId joint_create_revolute(b2WorldId world_handle,
		b2BodyId body_handle_1,
		b2BodyId body_handle_2,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		real_t angular_limit_lower,
		real_t angular_limit_upper,
		bool angular_limit_enabled,
		real_t motor_target_velocity,
		bool motor_enabled,
		bool disable_collision);

b2JointId joint_create_distance_joint(b2WorldId world_handle,
		b2BodyId body_handle_1,
		b2BodyId body_handle_2,
		const b2Vec2 anchor_1,
		const b2Vec2 anchor_2,
		real_t rest_length,
		real_t max_length,
		real_t stiffness,
		real_t damping,
		bool disable_collision);

void joint_change_distance_joint(b2WorldId world_handle,
		b2JointId joint_handle,
		real_t rest_length,
		real_t stiffness,
		real_t damping);

void joint_destroy(b2WorldId world_handle, b2JointId joint_handle);

ShapeCastResult shape_casting(b2WorldId world_handle,
		const b2Vec2 motion,
		ShapeInfo shape_info,
		bool collide_with_body,
		bool collide_with_area,
		QueryHandleExcludedCallback handle_excluded_callback,
		const QueryExcludedInfo *handle_excluded_info,
		double margin);

ShapeCollideResult shape_collide(const b2Vec2 motion1,
		ShapeInfo shape_info1,
		const b2Vec2 motion2,
		ShapeInfo shape_info2);

ShapeHandle shape_create_box(const b2Vec2 size);

ShapeHandle shape_create_capsule(real_t half_height, real_t radius);

ShapeHandle shape_create_circle(real_t radius, b2Vec2 = b2Vec2_zero);

ShapeHandle shape_create_concave_polyline(const b2Vec2 *points, size_t point_count);

ShapeHandle shape_create_convex_polyline(const b2Vec2 *points, size_t point_count);

ShapeHandle shape_create_halfspace(const b2Vec2 normal, real_t distance);

void shape_destroy(ShapeHandle shape_handle);

ContactResult shapes_contact(b2WorldId world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		real_t margin);

b2WorldId world_create(const WorldSettings *settings);

void world_destroy(b2WorldId world_handle);

size_t world_get_active_objects_count(b2World *world_handle);

void world_set_active_body_callback(b2World *world_handle, ActiveBodyCallback callback);

void world_set_collision_filter_callback(b2World *world_handle,
		b2ContactFilter *callback);

void world_set_contact_listener(b2WorldId world_handle,
		b2ContactListener *callback);

void world_step(b2WorldId world_handle, const SimulationSettings settings);

} // namespace box2d

#endif // BOX2D_WRAPPER_H
