#include "box2d_joint_2d.h"
#include "../spaces/box2d_space_2d.h"

Box2DJoint2D::Box2DJoint2D(Box2DBody2D *p_body_a, Box2DBody2D *p_body_b) {
	A = p_body_a;
	B = p_body_b;
}

void Box2DJoint2D::copy_settings_from(Box2DJoint2D *p_joint) {
	set_rid(p_joint->get_rid());
	set_max_force(p_joint->get_max_force());
	set_bias(p_joint->get_bias());
	set_max_bias(p_joint->get_max_bias());
}

void Box2DJoint2D::disable_collisions_between_bodies(const bool p_disabled) {
	disabled_collisions_between_bodies = p_disabled;
	if (box2d::is_handle_valid(handle)) {
		box2d::joint_set_disable_collision(handle, disabled_collisions_between_bodies);
	}
}

Box2DJoint2D::~Box2DJoint2D() {
	if (box2d::is_handle_valid(handle)) {
		ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));
		box2d::joint_destroy(space_handle, handle);
	}
};
