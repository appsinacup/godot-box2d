#ifndef BOX2D_GROOVE_JOINT_2D_H
#define BOX2D_GROOVE_JOINT_2D_H

#include "box2d_joint_2d.h"

class Box2DGrooveJoint2D : public Box2DJoint2D {
public:
	virtual PhysicsServer2D::JointType get_type() const override { return PhysicsServer2D::JOINT_TYPE_GROOVE; }

	Box2DGrooveJoint2D(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody2D *p_body_a, Box2DBody2D *p_body_b);
};

#endif // BOX2D_GROOVE_JOINT_2D_H
