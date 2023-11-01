#ifndef BOX2D_PIN_JOINT_2D_H
#define BOX2D_PIN_JOINT_2D_H

#include "box2d_joint_2d.h"

using namespace godot;

class Box2DPinJoint2D : public Box2DJoint2D {
	real_t angular_limit_lower = 0.0;
	real_t angular_limit_upper = 0.0;
	real_t motor_target_velocity = 0.0;
	bool motor_enabled = false;
	bool angular_limit_enabled = false;

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_PIN; }

	void set_param(PhysicsServer2D::PinJointParam p_param, real_t p_value);
	real_t get_param(PhysicsServer2D::PinJointParam p_param) const;

	void set_flag(PhysicsServer2D::PinJointFlag p_flag, bool p_enabled);
	bool get_flag(PhysicsServer2D::PinJointFlag p_flag) const;

	Box2DPinJoint2D(const Vector2 &p_pos, Box2DBody2D *p_body_a, Box2DBody2D *p_body_b = nullptr);
};

#endif // BOX2D_PIN_JOINT_2D_H
