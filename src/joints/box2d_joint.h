#pragma once

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <box2d/b2_joint.h>

using namespace godot;

class Box2DSpace;
class Box2DBody;

class Box2DJoint {
	RID self;
	struct Pin {
		real_t softness = 0;
	};
	struct DampedSpring {
		real_t rest_length = 0;
		real_t stiffness = 0;
		real_t damping = 0;
	};
	struct Groove {
		real_t lower_translation = 0;
		real_t upper_translation = 0;
		b2Vec2 axis;
	};
	struct CommonJoint {
		real_t bias = 0;
		real_t max_bias = 0;
		real_t max_force = 0;
		Box2DBody *body_a = nullptr;
		Box2DBody *body_b = nullptr;
		b2Vec2 anchor_a;
		b2Vec2 anchor_b;
	};
	DampedSpring damped_spring;
	Pin pin;
	Groove groove;
	CommonJoint common;

	b2Joint *joint = nullptr;
	b2JointDef *joint_def = nullptr;

	Box2DSpace *space = nullptr;

	void _recreate_joint();
	bool configured = false;
	PhysicsServer2D::JointType type;

public:
	_FORCE_INLINE_ PhysicsServer2D::JointType get_type() const { return type; }

	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	_FORCE_INLINE_ bool is_configured() const { return configured; }
	Box2DBody *get_body_a();
	Box2DBody *get_body_b();
	void set_bias(real_t p_data);
	void set_max_bias(real_t p_data);
	void set_max_force(real_t p_data);
	real_t get_bias();
	real_t get_max_bias();
	real_t get_max_force();
	Variant get_data() const;
	void clear();
	void set_disable_collisions(bool disable_collisions);
	bool get_disable_collisions();
	void make_pin(const Vector2 &p_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b);
	void make_groove(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b);
	void make_damped_spring(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody *p_body_a, Box2DBody *p_body_b);
	void set_pin_softness(real_t p_softness);
	real_t get_pin_softness();
	void set_damped_spring_rest_length(real_t p_damped_spring_rest_length);
	real_t get_damped_spring_rest_length();
	void set_damped_spring_stiffness(real_t p_damped_spring_stiffness);
	real_t get_damped_spring_stiffness();
	void set_damped_spring_damping(real_t p_damped_spring_damping);
	real_t get_damped_spring_damping();

	b2JointDef *get_b2JointDef();
	b2Joint *get_b2Joint();
	void set_b2Joint(b2Joint *p_joint);

	void set_space(Box2DSpace *p_space);

	Box2DJoint();
	virtual ~Box2DJoint();
};
