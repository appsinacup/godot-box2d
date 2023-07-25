#include "box2d_space_contact_listener.h"

#include "../b2_user_settings.h"

#include "../bodies/box2d_area.h"
#include "../bodies/box2d_collision_object.h"
#include "box2d/b2_contact.h"
#include <box2d/b2_shape.h>

void Box2DSpaceContactListener::handle_contact(b2Contact *contact, PhysicsServer2D::AreaBodyStatus status) {
	Box2DCollisionObject *bodyA = contact->GetFixtureA()->GetBody()->GetUserData().collision_object;
	Box2DCollisionObject *bodyB = contact->GetFixtureB()->GetBody()->GetUserData().collision_object;
	int shapeA = contact->GetFixtureA()->GetUserData().shape_idx;
	int shapeB = contact->GetFixtureA()->GetUserData().shape_idx;
	if (bodyA->get_type() == Box2DCollisionObject::Type::TYPE_AREA) {
		if (bodyB->get_type() == Box2DCollisionObject::Type::TYPE_AREA) {
			((Box2DArea *)bodyA)->call_area_monitor(((Box2DArea *)bodyB), status, bodyB->get_self(), bodyB->get_object_instance_id(), shapeB, shapeA);
		} else {
			((Box2DArea *)bodyA)->call_monitor(bodyB, status, bodyB->get_self(), bodyB->get_object_instance_id(), shapeB, shapeA);
		}
	} else if (bodyB->get_type() == Box2DCollisionObject::Type::TYPE_AREA) {
		((Box2DArea *)bodyB)->call_monitor(bodyA, status, bodyA->get_self(), bodyA->get_object_instance_id(), shapeA, shapeB);
	}
}

void Box2DSpaceContactListener::BeginContact(b2Contact *contact) {
	handle_contact(contact, PhysicsServer2D::AreaBodyStatus::AREA_BODY_ADDED);
}

void Box2DSpaceContactListener::EndContact(b2Contact *contact) {
	handle_contact(contact, PhysicsServer2D::AreaBodyStatus::AREA_BODY_REMOVED);
}

void Box2DSpaceContactListener::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
	b2Body *b2_body_A = contact->GetFixtureA()->GetBody();
	b2Body *b2_body_B = contact->GetFixtureB()->GetBody();
	Box2DCollisionObject *bodyA = b2_body_A->GetUserData().collision_object;
	Box2DCollisionObject *bodyB = b2_body_B->GetUserData().collision_object;
	int shapeA = contact->GetFixtureA()->GetUserData().shape_idx;
	int shapeB = contact->GetFixtureA()->GetUserData().shape_idx;
	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);
	int point_count = contact->GetManifold()->pointCount;
	if (b2_body_A->GetType() != b2BodyType::b2_dynamicBody && b2_body_B->GetType() == b2BodyType::b2_dynamicBody) {
		if (point_count > 0) {
			b2_body_B->ApplyLinearImpulse(b2_body_B->GetMass() * bodyA->get_constant_linear_velocity(), worldManifold.points[0], true);
		}
		float inertia = b2_body_B->GetInertia() - b2_body_B->GetMass() * b2Dot(b2_body_B->GetLocalCenter(), b2_body_B->GetLocalCenter());
		b2_body_B->ApplyTorque(inertia * bodyA->get_constant_angular_velocity(), true);
	}
	if (b2_body_B->GetType() != b2BodyType::b2_dynamicBody && b2_body_A->GetType() == b2BodyType::b2_dynamicBody) {
		if (point_count > 0) {
			b2_body_A->ApplyLinearImpulse(b2_body_A->GetMass() * bodyB->get_constant_linear_velocity(), worldManifold.points[0], true);
		}
		float inertia = b2_body_A->GetInertia() - b2_body_A->GetMass() * b2Dot(b2_body_A->GetLocalCenter(), b2_body_A->GetLocalCenter());
		b2_body_A->ApplyTorque(inertia * bodyB->get_constant_angular_velocity(), true);
	}
}

void Box2DSpaceContactListener::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
}
