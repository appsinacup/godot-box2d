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

void Box2DSpaceContactListener::handle_static_constant_linear_velocity(b2Body *b2_body_A, Box2DCollisionObject *bodyA, b2Body *b2_body_B, Box2DCollisionObject *bodyB, b2WorldManifold worldManifold, int points_count) {
	if (b2_body_A->GetType() != b2BodyType::b2_dynamicBody && b2_body_B->GetType() == b2BodyType::b2_dynamicBody) {
		Vector2 linear_velocity = bodyB->get_linear_velocity();
		bool linear_velocity_set = false;
		if (!is_zero(bodyA->get_constant_linear_velocity().x)) {
			linear_velocity.x = bodyA->get_constant_linear_velocity().x;
			linear_velocity_set = true;
		}
		if (!is_zero(bodyA->get_constant_linear_velocity().y)) {
			linear_velocity.y = bodyA->get_constant_linear_velocity().y;
			linear_velocity_set = true;
		}
		if (linear_velocity_set) {
			bodyB->set_linear_velocity(linear_velocity);
		}
		if (!is_zero(bodyA->get_constant_angular_velocity())) {
			bodyB->set_angular_velocity(bodyA->get_constant_angular_velocity());
		}
	}
}

bool Box2DSpaceContactListener::should_disable_collision_one_way_direction(b2Vec2 one_way_collision_direction_A, b2Body *b2_body_A, b2Body *b2_body_B, b2Vec2 body_B_velocity) {
	ERR_FAIL_COND_V(!b2_body_A, false);
	ERR_FAIL_COND_V(!b2_body_B, false);
	b2Vec2 normal = b2Mul(b2_body_A->GetTransform().q, one_way_collision_direction_A);
	b2Vec2 body_A_velocity = b2_body_A->GetLinearVelocity();

	body_B_velocity -= body_A_velocity;
	body_B_velocity.Normalize();
	float dot_product = b2Dot(body_B_velocity, normal);
	float passThroughThreshold = -b2_epsilon * 10;
	if (dot_product >= passThroughThreshold) {
		return true;
	}
	return false;
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
	bool one_way_collision_A = fixtureA_user_data.one_way_collision;
	bool one_way_collision_B = fixtureB_user_data.one_way_collision;
	b2Vec2 one_way_collision_direction_A(fixtureA_user_data.one_way_collision_direction_x, fixtureA_user_data.one_way_collision_direction_y);
	b2Vec2 one_way_collision_direction_B(fixtureB_user_data.one_way_collision_direction_x, fixtureB_user_data.one_way_collision_direction_y);
	contact->GetWorldManifold(&world_manifold);
	handle_static_constant_linear_velocity(b2_body_A, bodyA, b2_body_B, bodyB, world_manifold, contact->GetManifold()->pointCount);
	handle_static_constant_linear_velocity(b2_body_B, bodyB, b2_body_A, bodyA, world_manifold, contact->GetManifold()->pointCount);
	b2Vec2 b2_body_A_position = b2_body_A->GetPosition();
	b2Vec2 b2_body_B_position = b2_body_B->GetPosition();
	if (one_way_collision_A) {
		if (should_disable_collision_one_way_direction(one_way_collision_direction_A, b2_body_A, b2_body_B, b2_body_B->GetLinearVelocity())) {
			contact->SetEnabled(false);
			return;
		}
	}
	if (one_way_collision_B) {
		if (should_disable_collision_one_way_direction(one_way_collision_direction_B, b2_body_B, b2_body_A, b2_body_A->GetLinearVelocity())) {
			contact->SetEnabled(false);
		}
	}
}

void Box2DSpaceContactListener::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
}
