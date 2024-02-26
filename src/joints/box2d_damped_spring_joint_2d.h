#ifndef BOX2D_DAMPED_SPRING_JOINT_2D_H
#define BOX2D_DAMPED_SPRING_JOINT_2D_H

#include "box2d_joint_2d.h"

class Box2DDampedSpringJoint2D : public Box2DJoint2D {
	real_t rest_length = 0.0;
	real_t damping = 1.5;
	real_t stiffness = 20.0;

public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_DAMPED_SPRING; }

	void set_param(PhysicsServer2D::DampedSpringParam p_param, real_t p_value);
	real_t get_param(PhysicsServer2D::DampedSpringParam p_param) const;

	Box2DDampedSpringJoint2D(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody2D *p_body_a, Box2DBody2D *p_body_b);
	~Box2DDampedSpringJoint2D();
};

#endif // BOX2D_DAMPED_SPRING_JOINT_2D_H
