#include "box2d_damped_spring_joint_2d.h"
#include "../spaces/box2d_space_2d.h"

void Box2DDampedSpringJoint2D::set_param(PhysicsServer2D::DampedSpringParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer2D::DAMPED_SPRING_REST_LENGTH: {
			rest_length = p_value;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_DAMPING: {
			damping = p_value;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_STIFFNESS: {
			stiffness = p_value;
		} break;
	}
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
	box2d::joint_change_distance_joint(space_handle, handle, rest_length, stiffness, damping);
}

real_t Box2DDampedSpringJoint2D::get_param(PhysicsServer2D::DampedSpringParam p_param) const {
	switch (p_param) {
		case PhysicsServer2D::DAMPED_SPRING_REST_LENGTH: {
			return rest_length;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_DAMPING: {
			return damping;
		} break;
		case PhysicsServer2D::DAMPED_SPRING_STIFFNESS: {
			return stiffness;
		} break;
	}

	ERR_FAIL_V(0);
}

Box2DDampedSpringJoint2D::Box2DDampedSpringJoint2D(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody2D *p_body_a, Box2DBody2D *p_body_b) :
		Box2DJoint2D(p_body_a, p_body_b) {
	Vector2 anchor_A = A->get_inv_transform().xform(p_anchor_a);
	Vector2 anchor_B = B->get_inv_transform().xform(p_anchor_b);

	rest_length = p_anchor_a.distance_to(p_anchor_b);

	b2Vec2 box2d_anchor_A = { anchor_A.x, anchor_A.y };
	b2Vec2 box2d_anchor_B = { anchor_B.x, anchor_B.y };

	ERR_FAIL_COND(!p_body_a->get_space());
	ERR_FAIL_COND(p_body_a->get_space() != p_body_b->get_space());
	space_handle = p_body_a->get_space()->get_handle();
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));
	handle = box2d::joint_create_distance_joint(space_handle, p_body_a->get_body_handle(), p_body_b->get_body_handle(), box2d_anchor_A, box2d_anchor_B, rest_length, stiffness, damping, is_disabled_collisions_between_bodies());
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
}
