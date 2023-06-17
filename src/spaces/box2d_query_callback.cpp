#include "box2d_query_callback.h"

#include "../b2_user_settings.h"

Box2DQueryCallback::Box2DQueryCallback(PhysicsServer2DExtensionShapeResult *p_results,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		uint64_t p_canvas_instance_id,
		int32_t p_max_results) {
	results = p_results;
	collision_mask = p_collision_mask;
	collide_with_bodies = p_collide_with_bodies;
	collide_with_areas = p_collide_with_areas;
	canvas_instance_id = p_canvas_instance_id;
	max_results = p_max_results;
}

int32_t Box2DQueryCallback::get_hit_count() {
	return hit_count;
}

bool Box2DQueryCallback::ReportFixture(b2Fixture *fixture) {
	if ((fixture->GetFilterData().maskBits & collision_mask) != 0 &&
			((fixture->IsSensor() && collide_with_areas) ||
					(!fixture->IsSensor() && collide_with_bodies))) {
		hit_count++;
		PhysicsServer2DExtensionShapeResult &result = *results++;
		result.shape = fixture->GetUserData().shape_idx;
		Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
		result.rid = collision_object->get_self();
		result.collider_id = collision_object->get_object_instance_id();
		result.collider = collision_object->get_object();
	}
	return max_results - hit_count > 0;
}
