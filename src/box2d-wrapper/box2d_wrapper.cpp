#include "box2d_wrapper.h"
#include "box2d_conversion.h"
#include <box2d/box2d.h>
#include <box2d/hull.h>
#include <godot_cpp/templates/hash_set.hpp>

using namespace box2d;

enum class ShapeType {
	Circle = 0,
	Segment,
	Polygon,
};

Handle new_shape_handle(ShapeType type) {
	return Handle{-1,rand(),-1,static_cast<uint16_t>(type)};
}

godot::HashMap<Handle, b2Polygon> polygons;
godot::HashMap<Handle, b2Circle> circles;
godot::HashMap<Handle, b2Segment> segments;

Handle create_collider(Handle shape_handle, b2BodyId body_id, b2ShapeDef *shape_def) {
	switch(ShapeType(shape_handle.revision)) {
		case ShapeType::Circle: {
			ERR_FAIL_COND_V(!circles.has(shape_handle), invalid_handle());
			return shape_handle_to_handle(b2Body_CreateCircle(body_id, shape_def, &circles.get(shape_handle)));
		}
		case ShapeType::Polygon: {
			ERR_FAIL_COND_V(!polygons.has(shape_handle), invalid_handle());
			return shape_handle_to_handle(b2Body_CreatePolygon(body_id, shape_def, &polygons.get(shape_handle)));
		}
		case ShapeType::Segment: {
			ERR_FAIL_COND_V(!segments.has(shape_handle), invalid_handle());
			return shape_handle_to_handle(b2Body_CreateSegment(body_id, shape_def, &segments.get(shape_handle)));
		}
	}
	return invalid_handle();
}

void destroy_shape(Handle shape_handle) {
	switch(ShapeType(shape_handle.revision)) {
		case ShapeType::Circle: {
			circles.erase(shape_handle);
		} break;
		case ShapeType::Polygon: {
			polygons.erase(shape_handle);
		} break;
		case ShapeType::Segment: {
			segments.erase(shape_handle);
		} break;
	}
}

bool box2d::are_handles_equal(Handle handle1, Handle handle2) {
	return handle1.object_index == handle2.object_index &&
		handle1.revision == handle2.revision &&
		handle1.world == handle2.world &&
		handle1.world_index == handle2.world_index;
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
	b2BodyId body_id = handle_to_body_handle(body_handle);
	b2Body_SetType(body_id, body_type_to_b2_body_type(body_type));
	if (wakeup) {
		b2Body_Wake(body_id);
	}
}

Handle box2d::body_create(Handle world_handle,
		const Vector *pos,
		Real rot,
		const UserData *user_data,
		BodyType body_type) {
	b2BodyDef body_def = b2DefaultBodyDef();
	body_def.position = b2Vec2{pos->x, pos->y};
	body_def.angle = rot;
	body_def.userData = (void*)user_data;
	body_def.type = body_type_to_b2_body_type(body_type);
	b2BodyId body_id = b2World_CreateBody(handle_to_world_handle(world_handle), &body_def);
	return body_handle_to_handle(body_id);
}

void box2d::body_destroy(Handle world_handle, Handle body_handle) {
	b2World_DestroyBody(handle_to_body_handle(body_handle));
}

void box2d::body_force_sleep(Handle world_handle, Handle body_handle) {
}

Real box2d::body_get_angle(Handle world_handle, Handle body_handle) {
	return b2Body_GetAngle(handle_to_body_handle(body_handle));
}

Real box2d::body_get_angular_velocity(Handle world_handle, Handle body_handle) {
	return b2Body_GetAngularVelocity(handle_to_body_handle(body_handle));
}

Vector box2d::body_get_constant_force(Handle world_handle, Handle body_handle) {
	return Vector();
}

Real box2d::body_get_constant_torque(Handle world_handle, Handle body_handle) {
	return 0.0;
}

Vector box2d::body_get_linear_velocity(Handle world_handle, Handle body_handle) {
	return b2_vec_to_vector(b2Body_GetLinearVelocity(handle_to_body_handle(body_handle)));
}

Vector box2d::body_get_position(Handle world_handle, Handle body_handle) {
	return b2_vec_to_vector(b2Body_GetPosition(handle_to_body_handle(body_handle)));
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
	b2Body_SetAngularVelocity(handle_to_body_handle(body_handle), vel);
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
	b2Body_SetLinearVelocity(handle_to_body_handle(body_handle), vector_to_b2_vec(*vel));
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
	//b2Body_SetMassData(mass_data);
}

void box2d::body_set_transform(Handle world_handle,
		Handle body_handle,
		const Vector *pos,
		Real rot,
		bool wake_up) {
	b2BodyId body_id = handle_to_body_handle(body_handle);
	b2Body_SetTransform(body_id, vector_to_b2_vec(*pos), rot);
	if (wake_up) {
		b2Body_Wake(body_id);
	}
}

void box2d::body_update_material(Handle world_handle, Handle body_handle, const Material *mat) {
}

void box2d::body_wake_up(Handle world_handle, Handle body_handle, bool strong) {
	b2Body_Wake(handle_to_body_handle(body_handle));
}

Handle box2d::collider_create_sensor(Handle world_handle,
		Handle shape_handle,
		Handle body_handle,
		const UserData *user_data) {
	b2ShapeDef shape_def = b2DefaultShapeDef();
	shape_def.isSensor = true;
	shape_def.userData = (void*)user_data;
	b2BodyId body_id = handle_to_body_handle(body_handle);
	return create_collider(shape_handle, body_id, &shape_def);
}

Handle box2d::collider_create_solid(Handle world_handle,
		Handle shape_handle,
		const Material *mat,
		Handle body_handle,
		const UserData *user_data) {
	b2ShapeDef shape_def = b2DefaultShapeDef();
	shape_def.isSensor = false;
	shape_def.userData = (void*)user_data;
	b2BodyId body_id = handle_to_body_handle(body_handle);
	return create_collider(shape_handle, body_id, &shape_def);
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
	return QueryExcludedInfo{0,0,nullptr,0,0};
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
	return Handle{-1,-1,-1,0};
}

UserData box2d::invalid_user_data() {
	return UserData{uint64_t(-1), uint64_t(-1)};
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
	b2PrismaticJointDef joint_def = b2DefaultPrismaticJointDef();
	joint_def.bodyIdA = handle_to_body_handle(body_handle_1);
	joint_def.bodyIdB = handle_to_body_handle(body_handle_2);
	joint_def.localAxisA = vector_to_b2_vec(*axis);
	joint_def.localAnchorA = vector_to_b2_vec(*anchor_1);
	joint_def.localAnchorB = vector_to_b2_vec(*anchor_2);
	joint_def.lowerTranslation = limits->x;
	joint_def.upperTranslation = limits->y;
	b2JointId joint_id = b2World_CreatePrismaticJoint(handle_to_world_handle(world_handle), &joint_def);
	return joint_handle_to_handle(joint_id);
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
	b2RevoluteJointDef joint_def = b2DefaultRevoluteJointDef();
	joint_def.bodyIdA = handle_to_body_handle(body_handle_1);
	joint_def.bodyIdB = handle_to_body_handle(body_handle_2);
	joint_def.localAnchorA = vector_to_b2_vec(*anchor_1);
	joint_def.localAnchorB = vector_to_b2_vec(*anchor_2);
	joint_def.enableMotor = motor_enabled;
	joint_def.motorSpeed = motor_target_velocity;
	b2JointId joint_id = b2World_CreateRevoluteJoint(handle_to_world_handle(world_handle), &joint_def);
	return joint_handle_to_handle(joint_id);
}

void box2d::joint_destroy(Handle world_handle, Handle joint_handle) {
	b2World_DestroyJoint(handle_to_joint_handle(joint_handle));
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
	return polygons.insert(new_shape_handle(ShapeType::Polygon), b2MakeBox(size->x, size->y))->key;
}

Handle box2d::shape_create_capsule(Real half_height, Real radius) {
	return polygons.insert(new_shape_handle(ShapeType::Polygon), b2MakeCapsule(b2Vec2{0, -half_height}, b2Vec2{0, half_height}, radius))->key;
}

Handle box2d::shape_create_circle(Real radius) {
	return circles.insert(new_shape_handle(ShapeType::Circle), b2Circle{b2Vec2{0,0}, radius})->key;
}

Handle box2d::shape_create_convave_polyline(const Vector *points, size_t point_count) {
	return invalid_handle();
}

Handle box2d::shape_create_convex_polyline(const Vector *points, size_t point_count) {
	b2Vec2 b2_points[point_count];
	for (int i=0; i<point_count; i++) {
		b2_points[i] = vector_to_b2_vec(points[i]);
	}
	b2Hull hull = b2ComputeHull(b2_points, point_count);
	if (hull.count < 0) {
		return invalid_handle()
	}
	return polygons.insert(new_shape_handle(ShapeType::Polygon), b2MakePolygon(&hull, 0.0))->key;
}

Handle box2d::shape_create_halfspace(const Vector *normal) {
	return invalid_handle();
}

void box2d::shape_destroy(Handle shape_handle) {
	destroy_shape(shape_handle);
}

ContactResult box2d::shapes_contact(Handle world_handle,
		ShapeInfo shape_info1,
		ShapeInfo shape_info2,
		Real margin) {
	return ContactResult{};
}

Handle box2d::world_create(const WorldSettings *settings) {
	b2WorldDef world_def = b2DefaultWorldDef();
	b2WorldId handle = b2CreateWorld(&world_def);
	return world_handle_to_handle(handle);
}

void box2d::world_destroy(Handle world_handle) {
	b2DestroyWorld(handle_to_world_handle(world_handle));
}

size_t box2d::world_get_active_objects_count(Handle world_handle) {
	return b2World_GetStatistics(handle_to_world_handle(world_handle)).bodyCount;
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
	b2World_Step(handle_to_world_handle(world_handle), settings->dt, settings->max_velocity_iterations, settings-> max_velocity_iterations);
}
