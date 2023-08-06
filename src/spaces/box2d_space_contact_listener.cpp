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

bool Box2DSpaceContactListener::handle_static_constant_linear_velocity(b2Body *b2_body_A, Box2DCollisionObject *bodyA, b2Body *b2_body_B, Box2DCollisionObject *bodyB, b2Contact *contact) {
	if (b2_body_A->GetType() != b2BodyType::b2_dynamicBody && b2_body_B->GetType() == b2BodyType::b2_dynamicBody) {
		if (!world_manifold_computed) {
			contact->GetWorldManifold(&worldManifold);
		}
		int point_count = contact->GetManifold()->pointCount;
		if (point_count > 0) {
			b2_body_B->ApplyLinearImpulse(b2_body_B->GetMass() * bodyA->get_constant_linear_velocity(), worldManifold.points[0], true);
		}
		float inertia = b2_body_B->GetInertia() - b2_body_B->GetMass() * b2Dot(b2_body_B->GetLocalCenter(), b2_body_B->GetLocalCenter());
		b2_body_B->ApplyTorque(inertia * bodyA->get_constant_angular_velocity(), true);
		return true;
	}
	return false;
}

void Box2DSpaceContactListener::handle_one_way_direction(b2Vec2 one_way_collision_direction_A, b2Body *b2_body_A, b2Body *b2_body_B, b2Contact *contact) {
	if (!world_manifold_computed) {
		contact->GetWorldManifold(&worldManifold);
	}
	int point_count = contact->GetManifold()->pointCount;
	for (int i = 0; i < point_count; i++) {
		//b2Vec2 body_A_point_velocity = b2_body_A->GetLinearVelocityFromWorldPoint(worldManifold.points[i]);
		//b2Vec2 body_B_point_velocity = b2_body_B->GetLinearVelocityFromWorldPoint(worldManifold.points[i]);
		//b2Vec2 relative_velocity = b2_body_A->GetLocalVector(body_B_point_velocity - body_A_point_velocity);
		b2Vec2 localNormalA = b2_body_A->GetLocalVector(worldManifold.normal);
		float dot_product = b2Dot(localNormalA, one_way_collision_direction_A);
		if (dot_product <= 0.0f) {
			return;
		}
	}
	contact->SetEnabled(false);
}

void Box2DSpaceContactListener::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
	b2Fixture *fixtureA = contact->GetFixtureA();
	b2Fixture *fixtureB = contact->GetFixtureB();
	b2Body *b2_body_A = fixtureA->GetBody();
	b2Body *b2_body_B = fixtureB->GetBody();
	Box2DCollisionObject *bodyA = b2_body_A->GetUserData().collision_object;
	Box2DCollisionObject *bodyB = b2_body_B->GetUserData().collision_object;
	b2FixtureUserData fixtureA_user_data = fixtureA->GetUserData();
	b2FixtureUserData fixtureB_user_data = fixtureB->GetUserData();
	int one_way_collision_A = fixtureA_user_data.one_way_collision;
	int one_way_collision_B = fixtureB_user_data.one_way_collision;
	b2Vec2 one_way_collision_direction_A(fixtureA_user_data.one_way_collision_direction_x, fixtureA_user_data.one_way_collision_direction_y);
	b2Vec2 one_way_collision_direction_B(fixtureB_user_data.one_way_collision_direction_x, fixtureB_user_data.one_way_collision_direction_y);
	world_manifold_computed |= handle_static_constant_linear_velocity(b2_body_A, bodyA, b2_body_B, bodyB, contact);
	world_manifold_computed |= handle_static_constant_linear_velocity(b2_body_B, bodyB, b2_body_A, bodyA, contact);
	if (one_way_collision_A && b2_body_B->GetType() != b2BodyType::b2_staticBody) {
		handle_one_way_direction(-one_way_collision_direction_A, b2_body_A, b2_body_B, contact);
		return;
	}
	if (one_way_collision_B && b2_body_A->GetType() != b2BodyType::b2_staticBody) {
		handle_one_way_direction(-one_way_collision_direction_B, b2_body_B, b2_body_A, contact);
		return;
	}
}

void Box2DSpaceContactListener::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
}
