#include "box2d_space_contact_filter.h"

#include "../b2_user_settings.h"

#include "../bodies/box2d_collision_object.h"
#include <box2d/b2_fixture.h>

bool Box2DSpaceContactFilter::ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) {
	Box2DCollisionObject *bodyA = fixtureA->GetBody()->GetUserData().collision_object;
	Box2DCollisionObject *bodyB = fixtureA->GetBody()->GetUserData().collision_object;

	const b2Filter &filterA = fixtureA->GetFilterData();
	const b2Filter &filterB = fixtureB->GetFilterData();

	bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
	return collide && !bodyA->is_body_collision_excepted(bodyB);
}
