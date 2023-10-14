#include "box2d_query_callback.h"

#include "../b2_user_settings.h"

#define QUERY_MAX_SIZE 64

Box2DQueryCallback::Box2DQueryCallback(Box2DDirectSpaceState *p_space_state,
		uint32_t p_collision_layer,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas) {
	space_state = p_space_state;
	collision_mask = p_collision_mask;
	collision_layer = p_collision_layer;
	collide_with_bodies = p_collide_with_bodies;
	collide_with_areas = p_collide_with_areas;
}

Vector<b2Fixture *> Box2DQueryCallback::get_results() {
	return results;
}

bool Box2DQueryCallback::ReportFixture(b2Fixture *fixture) {
	if ( // collision mask or layer
			((fixture->GetFilterData().categoryBits & collision_mask) != 0 ||
					(fixture->GetFilterData().maskBits & collision_layer) != 0) &&
			// collide with area or body bit
			((fixture->IsSensor() && collide_with_areas) ||
					(!fixture->IsSensor() && collide_with_bodies))) {
		Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
		if (space_state->is_body_excluded_from_query(collision_object->get_self())) {
			return true;
		}
		results.append(fixture);
	}
	return results.size() < QUERY_MAX_SIZE;
}
