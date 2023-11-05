#include "box2d_groove_joint_2d.h"
#include "../spaces/box2d_space_2d.h"

Box2DGrooveJoint2D::Box2DGrooveJoint2D(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody2D *p_body_a, Box2DBody2D *p_body_b) :
		Box2DJoint2D(p_body_a, p_body_b) {
	Vector2 point_A_1 = A->get_inv_transform().xform(p_a_groove1);
	Vector2 point_A_2 = A->get_inv_transform().xform(p_a_groove2);

	Vector2 anchor_B = B->get_inv_transform().xform(p_b_anchor);

	Vector2 axis = (point_A_2 - point_A_1).normalized();
	real_t length = (point_A_2 - point_A_1).length();

	b2Vec2 box2d_anchor_A = { point_A_1.x, point_A_1.y };
	b2Vec2 box2d_anchor_B = { anchor_B.x, anchor_B.y };

	b2Vec2 box2d_axis = { axis.x, axis.y };
	b2Vec2 box2d_limits = { 0.0, length };

	ERR_FAIL_COND(!p_body_a->get_space());
	ERR_FAIL_COND(p_body_a->get_space() != p_body_b->get_space());
	space_handle = p_body_a->get_space()->get_handle();
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

	handle = box2d::joint_create_prismatic(space_handle, p_body_a->get_body_handle(), p_body_b->get_body_handle(), box2d_axis, box2d_anchor_A, box2d_anchor_B, box2d_limits);
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));

	//A->add_constraint(this, 0);
	//B->add_constraint(this, 1);
}
