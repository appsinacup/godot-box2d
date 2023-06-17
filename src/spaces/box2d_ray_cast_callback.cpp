#include "box2d_ray_cast_callback.h"

#include "../b2_user_settings.h"

Box2DRayCastCallback::Box2DRayCastCallback(PhysicsServer2DExtensionRayResult *p_result,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		bool p_hit_from_inside) {
	result = p_result;
	collision_mask = p_collision_mask;
	collide_with_bodies = p_collide_with_bodies;
	collide_with_areas = p_collide_with_areas;
	hit_from_inside = p_hit_from_inside;
}

bool Box2DRayCastCallback::get_hit() {
	return hit;
}

float Box2DRayCastCallback::ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
		const b2Vec2 &normal, float fraction) {
	if ((fixture->GetFilterData().maskBits & collision_mask) != 0 &&
			((fixture->IsSensor() && collide_with_areas) ||
					(!fixture->IsSensor() && collide_with_bodies))) {
		result->normal = box2d_to_godot(normal);
		result->position = box2d_to_godot(point);
		result->shape = fixture->GetUserData().shape_idx;
		Box2DCollisionObject *collision_object = fixture->GetBody()->GetUserData().collision_object;
		result->rid = collision_object->get_self();
		result->collider_id = collision_object->get_object_instance_id();
		result->collider = collision_object->get_object_unsafe();
		hit = true;
		return 0;
	}
	return 1;
}
