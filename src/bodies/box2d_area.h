#pragma once

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "../spaces/box2d_space.h"
#include "box2d_collision_object.h"

using namespace godot;

class Box2DBody;

class Box2DArea : public Box2DCollisionObject {
	bool monitorable = false;
	Callable monitor_callback;
	Callable area_monitor_callback;
	struct Gravity {
		PhysicsServer2D::AreaSpaceOverrideMode override_mode = PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED;
		real_t gravity = 9.8;
		b2Vec2 vector = b2Vec2(0, 1);
		bool is_point = false;
		real_t point_unit_distance = 0;
	};
	Gravity gravity;
	PhysicsServer2D::AreaSpaceOverrideMode linear_damp_override_mode = PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED;
	PhysicsServer2D::AreaSpaceOverrideMode angular_damp_override_mode = PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_DISABLED;
	Vector<Box2DCollisionObject *> bodies;

	void update_bodies();

public:
	virtual void set_linear_damp(real_t p_linear_damp) override;
	virtual void set_angular_damp(real_t p_angular_damp) override;
	virtual void set_priority(real_t p_priority) override;
	// Physics Server
	void set_monitorable(bool monitorable);
	bool get_monitorable();
	bool get_monitoring();
	void set_monitor_callback(const Callable &callback);
	void set_area_monitor_callback(const Callable &callback);
	void call_area_monitor(Box2DArea *area, PhysicsServer2D::AreaBodyStatus status, const RID &p_area, ObjectID p_instance, int area_shape_idx, int self_shape_idx);
	void call_monitor(Box2DCollisionObject *body, PhysicsServer2D::AreaBodyStatus status, const RID &p_body, ObjectID p_instance, int32_t area_shape_idx, int32_t self_shape_idx);

	virtual void set_transform(const Transform2D &p_transform) override;

	virtual void set_space(Box2DSpace *p_space) override;

	void set_gravity_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value);
	void set_gravity(real_t p_value);
	void set_gravity_vector(Vector2 p_value);
	void set_gravity_is_point(bool p_value);
	void set_gravity_point_unit_distance(double p_value);
	void set_linear_damp_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value);
	void set_angular_damp_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value);

	PhysicsServer2D::AreaSpaceOverrideMode get_gravity_override_mode() const;
	double get_gravity() const;
	b2Vec2 get_b2_gravity(Transform2D transform) const;
	Vector2 get_gravity_vector() const;
	bool get_gravity_is_point() const;
	double get_gravity_point_unit_distance() const;
	PhysicsServer2D::AreaSpaceOverrideMode get_linear_damp_override_mode() const;
	PhysicsServer2D::AreaSpaceOverrideMode get_angular_damp_override_mode() const;

	void add_body(Box2DCollisionObject *p_body);
	void remove_body(Box2DCollisionObject *p_body);

	Box2DArea();
	~Box2DArea();
};
