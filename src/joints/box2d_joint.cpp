#include "box2d_joint.h"
#include "bodies/box2d_body.h"
#include "box2d_type_conversions.h"
#include "spaces/box2d_space.h"

#include <box2d/b2_distance_joint.h>
#include <box2d/b2_prismatic_joint.h>
#include <box2d/b2_revolute_joint.h>
#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DJoint::clear() {
	configured = false;
	common.body_a = nullptr;
	common.body_b = nullptr;
	if (space) {
		space->remove_joint(this);
	}
}

void Box2DJoint::_recreate_joint() {
	if (space) {
		space->create_joint(this);
	}
}

void Box2DJoint::set_bias(real_t p_data) {
	common.bias = p_data;
}
void Box2DJoint::set_max_bias(real_t p_data) {
	common.max_bias = p_data;
}
void Box2DJoint::set_max_force(real_t p_data) {
	common.max_force = p_data;
}
real_t Box2DJoint::get_bias() {
	return common.bias;
}
real_t Box2DJoint::get_max_bias() {
	return common.max_bias;
}
real_t Box2DJoint::get_max_force() {
	return common.max_force;
}

void Box2DJoint::set_disable_collisions(bool p_disable_collisions) {
	if (joint_def->collideConnected == !p_disable_collisions) {
		return;
	}
	joint_def->collideConnected = !p_disable_collisions;
	_recreate_joint();
}

bool Box2DJoint::get_disable_collisions() {
	return !joint_def->collideConnected;
}

void Box2DJoint::make_pin(const Vector2 &p_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_PIN;
	common.anchor_a = godot_to_box2d(p_anchor);
	b2RevoluteJointDef *revolute_joint_def = memnew(b2RevoluteJointDef);
	revolute_joint_def->collideConnected = joint_def->collideConnected;
	memdelete(joint_def);
	joint_def = revolute_joint_def;
	common.body_a = p_body_a;
	common.body_b = p_body_b;
	// body_a and body_b are set when joint is created
	configured = true;
}

void Box2DJoint::make_groove(const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_GROOVE;
	groove.lower_translation = godot_to_box2d((p_a_groove2 - p_b_anchor).length());
	groove.upper_translation = godot_to_box2d((p_a_groove1 - p_b_anchor).length());
	Vector2 axis = (p_a_groove1 - p_a_groove2).normalized();
	groove.axis = b2Vec2(axis.x, axis.y);
	common.anchor_b = godot_to_box2d(p_b_anchor);
	b2PrismaticJointDef *prismatic_joint_def = memnew(b2PrismaticJointDef);
	prismatic_joint_def->collideConnected = joint_def->collideConnected;
	memdelete(joint_def);
	joint_def = prismatic_joint_def;
	common.body_a = p_body_a;
	common.body_b = p_body_b;
	// body_a and body_b are set when joint is created
	configured = true;
}

void Box2DJoint::make_damped_spring(const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, Box2DBody *p_body_a, Box2DBody *p_body_b) {
	type = PhysicsServer2D::JointType::JOINT_TYPE_DAMPED_SPRING;
	common.anchor_a = godot_to_box2d(p_anchor_a);
	common.anchor_b = godot_to_box2d(p_anchor_b);
	b2DistanceJointDef *distance_joint_def = memnew(b2DistanceJointDef);
	distance_joint_def->collideConnected = joint_def->collideConnected;
	memdelete(joint_def);
	joint_def = distance_joint_def;
	common.body_a = p_body_a;
	common.body_b = p_body_b;
	// body_a and body_b are set when joint is created
	configured = true;
}

void Box2DJoint::set_pin_softness(real_t p_softness) {
	pin.softness = p_softness; // unused
}

real_t Box2DJoint::get_pin_softness() {
	return pin.softness;
}

void Box2DJoint::set_damped_spring_rest_length(real_t p_damped_spring_rest_length) {
	if (damped_spring.rest_length == godot_to_box2d(p_damped_spring_rest_length)) {
		return;
	}
	damped_spring.rest_length = godot_to_box2d(p_damped_spring_rest_length);
	if (joint && (b2DistanceJoint *)joint) {
		b2DistanceJoint *distance_joint = (b2DistanceJoint *)joint;
		distance_joint->SetLength(damped_spring.rest_length);
	}
}
real_t Box2DJoint::get_damped_spring_rest_length() {
	return box2d_to_godot(damped_spring.rest_length);
}

void Box2DJoint::set_damped_spring_stiffness(real_t p_damped_spring_stiffness) {
	if (damped_spring.stiffness == godot_to_box2d(p_damped_spring_stiffness)) {
		return;
	}
	damped_spring.stiffness = godot_to_box2d(p_damped_spring_stiffness);
	if (joint && (b2DistanceJoint *)joint && common.body_a && common.body_b) {
		b2DistanceJoint *distance_joint = (b2DistanceJoint *)joint;
		float stiffness = 0;
		float damping = 0;
		b2LinearStiffness(stiffness, damping, damped_spring.stiffness, damped_spring.damping, common.body_a->get_b2Body(), common.body_b->get_b2Body());
		distance_joint->SetStiffness(stiffness);
	}
}
real_t Box2DJoint::get_damped_spring_stiffness() {
	return box2d_to_godot(damped_spring.stiffness);
}

void Box2DJoint::set_damped_spring_damping(real_t p_damped_spring_damping) {
	if (damped_spring.damping == godot_to_box2d(p_damped_spring_damping)) {
		return;
	}
	damped_spring.damping = godot_to_box2d(p_damped_spring_damping);
	if (joint && (b2DistanceJoint *)joint && common.body_a && common.body_b) {
		b2DistanceJoint *distance_joint = (b2DistanceJoint *)joint;
		float stiffness = 0;
		float damping = 0;
		b2LinearStiffness(stiffness, damping, damped_spring.stiffness, damped_spring.damping, common.body_a->get_b2Body(), common.body_b->get_b2Body());
		distance_joint->SetStiffness(damping);
	}
}
real_t Box2DJoint::get_damped_spring_damping() {
	return box2d_to_godot(damped_spring.damping);
}

Box2DBody *Box2DJoint::get_body_a() {
	return common.body_a;
}
Box2DBody *Box2DJoint::get_body_b() {
	return common.body_b;
}

b2JointDef *Box2DJoint::get_b2JointDef() {
	switch (type) {
		case PhysicsServer2D::JointType::JOINT_TYPE_PIN: {
			b2RevoluteJointDef *revolute_joint_def = (b2RevoluteJointDef *)joint_def;
			revolute_joint_def->enableMotor = true;
			revolute_joint_def->Initialize(common.body_a->get_b2Body(), common.body_b->get_b2Body(), common.anchor_a);
		} break;
		case PhysicsServer2D::JointType::JOINT_TYPE_DAMPED_SPRING: {
			b2DistanceJointDef *distance_joint_def = (b2DistanceJointDef *)joint_def;
			b2LinearStiffness(distance_joint_def->stiffness, distance_joint_def->damping, damped_spring.stiffness, damped_spring.damping, common.body_a->get_b2Body(), common.body_b->get_b2Body());
			distance_joint_def->Initialize(common.body_a->get_b2Body(), common.body_b->get_b2Body(), common.anchor_a, common.anchor_b);

			distance_joint_def->length = damped_spring.rest_length;
			distance_joint_def->minLength = 0;
		} break;
		case PhysicsServer2D::JointType::JOINT_TYPE_GROOVE: {
			b2PrismaticJointDef *prismatic_joint_def = (b2PrismaticJointDef *)joint_def;
			prismatic_joint_def->Initialize(common.body_a->get_b2Body(), common.body_b->get_b2Body(), common.anchor_b, groove.axis);
			prismatic_joint_def->lowerTranslation = -groove.lower_translation;
			prismatic_joint_def->upperTranslation = groove.upper_translation;
			prismatic_joint_def->enableLimit = true;
		} break;
		default: {
			ERR_PRINT("UNSUPPORTED");
		}
	}
	return joint_def;
}
b2Joint *Box2DJoint::get_b2Joint() {
	return joint;
}
void Box2DJoint::set_b2Joint(b2Joint *p_joint) {
	joint = p_joint;
}

void Box2DJoint::set_space(Box2DSpace *p_space) {
	space = p_space;
}

Box2DJoint::Box2DJoint() {
	joint_def = memnew(b2JointDef);
}

Box2DJoint::~Box2DJoint() {
	memdelete(joint_def);
}
