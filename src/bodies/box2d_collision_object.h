#pragma once

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>

#include "../box2d_type_conversions.h"
#include "../shapes/box2d_shape.h"
#include "../spaces/box2d_space.h"

using namespace godot;

class Box2DDirectSpaceState;
class Box2DJoint;
class Box2DArea;

class Box2DCollisionObject {
public:
	enum Type {
		TYPE_AREA,
		TYPE_BODY
	};
	bool operator<(const Box2DCollisionObject &other) const {
		return collision.priority < other.collision.priority;
	}

protected:
	Type type;
	RID self;
	ObjectID object_instance_id;
	ObjectID canvas_instance_id;

	b2Body *body = nullptr;
	b2BodyDef *body_def = nullptr;
	Box2DSpace *space = nullptr;
	HashSet<Box2DCollisionObject *> collision_exception;
	Vector<Box2DArea *> areas;

	struct Shape {
		Transform2D xform;
		Box2DShape *shape = nullptr;
		Vector<b2Fixture *> fixtures;
		bool disabled = false;
		bool one_way_collision = false;
	};
	Vector<Shape> shapes;

	struct Collision {
		real_t priority = 1;
		bool pickable = false;
	};
	b2Filter filter;
	Collision collision;
	struct ConstantForces {
		b2Vec2 constant_force = b2Vec2_zero;
		b2Vec2 constant_force_position = b2Vec2_zero;
		real_t constant_torque = 0;
	};
	ConstantForces constant_forces;
	struct PhysicsMaterial {
		real_t friction = 1;
		real_t bounce = 0;
	};
	PhysicsMaterial physics_material;
	struct Damping {
		real_t linear_damp = 0;
		real_t angular_damp = 0;
		PhysicsServer2D::BodyDampMode linear_damp_mode = PhysicsServer2D::BodyDampMode::BODY_DAMP_MODE_COMBINE;
		PhysicsServer2D::BodyDampMode angular_damp_mode = PhysicsServer2D::BodyDampMode::BODY_DAMP_MODE_COMBINE;
	};
	Damping damping;
	real_t total_linear_damp = 0.1;
	real_t total_angular_damp = 1;
	b2Vec2 total_gravity = b2Vec2(0, -9.8);

	void _clear_fixtures();
	void _update_shapes();
	Box2DDirectSpaceState *direct_space = nullptr;

	b2MassData mass_data;
	real_t gravity_scale = 1;
	struct ContactEdgeData {
		b2ContactEdge *edge;
		int32_t point_idx;
	};

	ContactEdgeData _get_contact_edge_data(int32_t contact_idx) const;

protected:
	void _set_transform(const Transform2D &p_transform, bool p_update_shapes = true);

	void _set_space(Box2DSpace *p_space);

	Box2DCollisionObject(Type p_type);

public:
	void recreate_shapes();
	void set_linear_damp_mode(PhysicsServer2D::BodyDampMode p_linear_damp);
	virtual void set_linear_damp(real_t p_linear_damp);
	void set_angular_damp_mode(PhysicsServer2D::BodyDampMode p_linear_damp);
	virtual void set_angular_damp(real_t p_angular_damp);
	virtual void set_priority(real_t p_priority);
	void set_bounce(real_t p_bounce);
	void set_friction(real_t p_friction);
	void set_mass(real_t p_mass);
	void set_inertia(real_t p_inertia);
	void set_center_of_mass(Vector2 p_center_of_mass);

	PhysicsServer2D::BodyDampMode get_linear_damp_mode() const;
	double get_linear_damp() const;
	PhysicsServer2D::BodyDampMode get_angular_damp_mode() const;
	double get_angular_damp() const;
	double get_priority() const;
	double get_bounce() const;
	double get_friction() const;
	double get_mass() const;
	double get_inertia() const;
	void reset_mass_properties();

	// Direct Body API
	virtual Vector2 get_total_gravity() const;
	virtual double get_total_linear_damp() const;
	virtual double get_total_angular_damp() const;
	virtual Vector2 get_center_of_mass() const;
	virtual Vector2 get_center_of_mass_local() const;
	virtual double get_inverse_mass() const;
	virtual double get_inverse_inertia() const;
	virtual void set_linear_velocity(const Vector2 &velocity);
	virtual Vector2 get_linear_velocity() const;
	virtual void set_angular_velocity(double velocity);
	virtual double get_angular_velocity() const;
	virtual void set_transform(const Transform2D &transform);
	virtual Transform2D get_transform() const;
	virtual Vector2 get_velocity_at_local_position(const Vector2 &local_position) const;
	virtual void apply_central_impulse(const Vector2 &impulse);
	virtual void apply_impulse(const Vector2 &impulse, const Vector2 &position);
	virtual void apply_torque_impulse(double impulse);
	virtual void apply_central_force(const Vector2 &force);
	virtual void apply_force(const Vector2 &force, const Vector2 &position);
	virtual void apply_torque(double torque);
	virtual void add_constant_central_force(const Vector2 &force);
	virtual void add_constant_force(const Vector2 &force, const Vector2 &position);
	virtual void add_constant_torque(double torque);
	virtual void set_constant_force(const Vector2 &force);
	virtual Vector2 get_constant_force() const;
	virtual void set_constant_torque(double torque);
	virtual double get_constant_torque() const;
	virtual void set_sleep_state(bool enabled);
	virtual bool is_sleeping() const;
	virtual int32_t get_contact_count() const;
	virtual Vector2 get_contact_local_position(int32_t contact_idx) const;
	virtual Vector2 get_contact_local_normal(int32_t contact_idx) const;
	virtual int32_t get_contact_local_shape(int32_t contact_idx) const;
	virtual RID get_contact_collider(int32_t contact_idx) const;
	virtual Vector2 get_contact_collider_position(int32_t contact_idx) const;
	virtual uint64_t get_contact_collider_id(int32_t contact_idx) const;
	virtual Object *get_contact_collider_object(int32_t contact_idx) const;
	virtual int32_t get_contact_collider_shape(int32_t contact_idx) const;
	virtual Vector2 get_contact_collider_velocity_at_position(int32_t contact_idx) const;
	virtual Vector2 get_contact_impulse(int32_t contact_idx) const;
	virtual double get_step() const;
	virtual void integrate_forces();
	virtual PhysicsDirectSpaceState2D *get_space_state();

	// Physics Server

	virtual void set_object_instance_id(const ObjectID &p_instance_id);
	virtual ObjectID get_object_instance_id() const;
	virtual void set_canvas_instance_id(const ObjectID &p_instance_id);
	virtual ObjectID get_canvas_instance_id() const;
	virtual void set_collision_layer(uint32_t layer);
	virtual uint32_t get_collision_layer() const;
	virtual void set_collision_mask(uint32_t layer);
	virtual uint32_t get_collision_mask() const;
	virtual void set_pickable(bool pickable);
	virtual Box2DSpace *get_space() const;
	virtual void add_shape(Box2DShape *p_shape, const Transform2D &p_transform = Transform2D(), bool p_disabled = false);
	virtual void set_shape(int p_index, Box2DShape *p_shape);
	virtual void set_shape_transform(int p_index, const Transform2D &p_transform);
	virtual void set_shape_disabled(int p_index, bool p_disabled);
	virtual void set_shape_as_one_way_collision(int p_index, bool enable);
	virtual int get_shape_count() const;
	virtual Box2DShape *get_shape(int p_index) const;
	virtual const Transform2D &get_shape_transform(int p_index) const;
	virtual void remove_shape(Box2DShape *p_shape);
	virtual void remove_shape(int p_index);
	void set_gravity_scale(real_t p_gravity_scale);
	real_t get_gravity_scale();
	void add_collision_exception(Box2DCollisionObject *excepted_body);
	void remove_collision_exception(Box2DCollisionObject *excepted_body);
	bool is_body_collision_excepted(Box2DCollisionObject *excepted_body);

	TypedArray<RID> get_collision_exception();

	virtual void set_space(Box2DSpace *p_space) = 0;

	// MISC
	void sort_areas();
	void recalculate_total_gravity();
	void recalculate_total_linear_damp();
	void recalculate_total_angular_damp();

	Object *get_object() const;
	Object *get_object_unsafe() const;
	Type get_type() const;
	void set_self(const RID &p_self);
	RID get_self() const;
	virtual void add_area(Box2DArea *p_area);
	virtual void remove_area(Box2DArea *p_area);

	b2BodyDef *get_b2BodyDef();
	void set_b2BodyDef(b2BodyDef *p_body_def);
	b2Body *get_b2Body();
	virtual void set_b2Body(b2Body *p_body);
	virtual HashSet<Box2DJoint *> get_joints() { return HashSet<Box2DJoint *>(); }

	void before_step();
	Box2DCollisionObject();
	virtual ~Box2DCollisionObject();
};
