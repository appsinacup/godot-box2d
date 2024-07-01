#include "box2d_pin_joint_2d.h"
#include "../spaces/box2d_space_2d.h"

void Box2DPinJoint2D::set_param(PhysicsServer2D::PinJointParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer2D::PIN_JOINT_LIMIT_UPPER: {
			angular_limit_upper = p_value;
		} break;
		case PhysicsServer2D::PIN_JOINT_LIMIT_LOWER: {
			angular_limit_lower = p_value;
		} break;
		case PhysicsServer2D::PIN_JOINT_MOTOR_TARGET_VELOCITY: {
			motor_target_velocity = p_value;
		} break;
		case PhysicsServer2D::PIN_JOINT_SOFTNESS: {
			return;
		}
	}
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
	box2d::joint_change_revolute_params(handle, angular_limit_lower, angular_limit_upper, angular_limit_enabled, motor_target_velocity, motor_enabled);
}

real_t Box2DPinJoint2D::get_param(PhysicsServer2D::PinJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer2D::PIN_JOINT_LIMIT_UPPER: {
			return angular_limit_upper;
		}
		case PhysicsServer2D::PIN_JOINT_LIMIT_LOWER: {
			return angular_limit_lower;
		}
		case PhysicsServer2D::PIN_JOINT_MOTOR_TARGET_VELOCITY: {
			return motor_target_velocity;
		}
		case PhysicsServer2D::PIN_JOINT_SOFTNESS: {
			return 0.0;
		}
	}
	ERR_FAIL_V(0);
}

void Box2DPinJoint2D::set_flag(PhysicsServer2D::PinJointFlag p_flag, bool p_enabled) {
	switch (p_flag) {
		case PhysicsServer2D::PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED: {
			angular_limit_enabled = p_enabled;
		} break;
		case PhysicsServer2D::PIN_JOINT_FLAG_MOTOR_ENABLED: {
			motor_enabled = p_enabled;
		} break;
	}
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
	box2d::joint_change_revolute_params(handle, angular_limit_lower, angular_limit_upper, angular_limit_enabled, motor_target_velocity, motor_enabled);
}

bool Box2DPinJoint2D::get_flag(PhysicsServer2D::PinJointFlag p_flag) const {
	switch (p_flag) {
		case PhysicsServer2D::PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED: {
			return angular_limit_enabled;
		}
		case PhysicsServer2D::PIN_JOINT_FLAG_MOTOR_ENABLED: {
			return motor_enabled;
		}
	}
	ERR_FAIL_V(0);
}

Box2DPinJoint2D::Box2DPinJoint2D(const Vector2 &p_pos, Box2DBody2D *p_body_a, Box2DBody2D *p_body_b) :
		Box2DJoint2D(p_body_a, p_body_b) {
	Vector2 anchor_A = p_body_a->get_inv_transform().xform(p_pos);
	Vector2 anchor_B = p_body_b ? p_body_b->get_inv_transform().xform(p_pos) : p_pos;

	b2Vec2 box2d_anchor_A = { anchor_A.x, anchor_A.y };
	b2Vec2 box2d_anchor_B = { anchor_B.x, anchor_B.y };

	ERR_FAIL_COND(!p_body_a->get_space());
	ERR_FAIL_COND(p_body_a->get_space() != p_body_b->get_space());
	b2WorldId space_handle = p_body_a->get_space()->get_handle();
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));
	handle = box2d::joint_create_revolute(space_handle, p_body_a->get_body_handle(), p_body_b->get_body_handle(), box2d_anchor_A, box2d_anchor_B, angular_limit_lower, angular_limit_upper, angular_limit_enabled, motor_target_velocity, motor_enabled, is_disabled_collisions_between_bodies());
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
}
