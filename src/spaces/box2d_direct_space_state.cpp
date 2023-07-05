#include "box2d_direct_space_state.h"

#include "../b2_user_settings.h"

#include "../bodies/box2d_collision_object.h"
#include "../box2d_type_conversions.h"
#include "box2d_query_callback.h"
#include "box2d_ray_cast_callback.h"

#include <box2d/b2_collision.h>
#include <box2d/b2_fixture.h>

PhysicsDirectSpaceState2D *Box2DDirectSpaceState::get_space_state() {
	ERR_FAIL_NULL_V(space, nullptr);
	return space->get_direct_state();
}

bool Box2DDirectSpaceState::_intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *result) {
	Box2DRayCastCallback callback(result, collision_mask, collide_with_bodies, collide_with_areas, hit_from_inside);
	space->get_b2World()->RayCast(&callback, godot_to_box2d(from), godot_to_box2d(to));
	return callback.get_hit();
}
int32_t Box2DDirectSpaceState::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *results, int32_t max_results) {
	/*
	Box2DQueryCallback callback(results, collision_mask, collide_with_bodies, collide_with_areas, canvas_instance_id, max_results);
	b2Vec2 pos(godot_to_box2d(position));
	b2AABB aabb;
	float point_size = 1.0f;

	aabb.lowerBound.Set(pos.x - point_size, pos.y - point_size);
	aabb.upperBound.Set(pos.x + point_size, pos.y + point_size);
	space->get_b2World()->QueryAABB(&callback, aabb);
	return callback.get_hit_count();
	*/
	// TODO reenable this when I figure out how to send Object* from this method
	return 0;
}
int32_t Box2DDirectSpaceState::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *result, int32_t max_results) {
	return 0;
}
bool Box2DDirectSpaceState::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *closest_safe, float *closest_unsafe) {
	return false;
}
bool Box2DDirectSpaceState::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	return false;
}
bool Box2DDirectSpaceState::_rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *rest_info) {
	return false;
}
