#include "box2d_direct_space_state_2d.h"

int Box2DDirectSpaceState2D::_intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) {
	ERR_FAIL_COND_V(space->locked, 0);
	ERR_FAIL_COND_V(!box2d::is_handle_valid(space->handle), 0);
	ERR_FAIL_COND_V(p_result_max < 0, 0);

	b2Vec2 box2d_pos = { position.x, position.y };

	box2d::PointHitInfo *hit_info_array = (box2d::PointHitInfo *)memalloc(p_result_max * sizeof(box2d::PointHitInfo));

	box2d::QueryExcludedInfo query_excluded_info = box2d::default_query_excluded_info();
	query_excluded_info.query_canvas_instance_id = canvas_instance_id;
	query_excluded_info.query_collision_layer_mask = collision_mask;

	uint32_t result_count = box2d::intersect_point(space->handle, box2d_pos, collide_with_bodies, collide_with_areas, hit_info_array, p_result_max, Box2DSpace2D::_is_handle_excluded_callback, &query_excluded_info);
	ERR_FAIL_COND_V(result_count > (uint32_t)p_result_max, 0);

	for (uint32_t i = 0; i < result_count; i++) {
		box2d::PointHitInfo &hit_info = hit_info_array[i];
		PhysicsServer2DExtensionShapeResult &result = r_results[i];
		ERR_CONTINUE(!box2d::is_user_data_valid(hit_info.user_data));

		uint32_t shape_index = 0;
		Box2DCollisionObject2D *collision_object_2d = Box2DCollisionObject2D::get_collider_user_data(hit_info.user_data, shape_index);
		ERR_CONTINUE(collision_object_2d == nullptr);

		result.shape = shape_index;
		result.collider_id = collision_object_2d->get_instance_id();
		result.rid = collision_object_2d->get_rid();

		if (result.collider_id.is_valid()) {
			result.collider = Box2DSpace2D::_get_object_instance_hack(result.collider_id);
		}
	}

	return result_count;
}

bool Box2DDirectSpaceState2D::_intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *r_result) {
	ERR_FAIL_COND_V(space->locked, false);
	ERR_FAIL_COND_V(!box2d::is_handle_valid(space->handle), false);

	// Raycast Info
	Vector2 begin, end, dir;
	begin = from;
	end = (to - from);
	dir = end.normalized();
	real_t length = end.length();

	b2Vec2 box2d_from = { begin.x, begin.y };
	b2Vec2 box2d_dir = { dir.x, dir.y };

	box2d::QueryExcludedInfo query_excluded_info = box2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;

	box2d::RayHitInfo hit_info;
	bool collide = box2d::intersect_ray(space->handle,
			box2d_from,
			box2d_dir,
			length,
			collide_with_bodies,
			collide_with_areas,
			hit_from_inside,
			&hit_info,
			Box2DSpace2D::_is_handle_excluded_callback,
			&query_excluded_info);

	if (collide) {
		r_result->position = Vector2(hit_info.position.x, hit_info.position.y);
		r_result->normal = Vector2(hit_info.normal.x, hit_info.normal.y);

		ERR_FAIL_COND_V(!box2d::is_user_data_valid(hit_info.user_data), false);
		uint32_t shape_index = 0;
		Box2DCollisionObject2D *collision_object_2d = Box2DCollisionObject2D::get_collider_user_data(hit_info.user_data, shape_index);
		ERR_FAIL_NULL_V(collision_object_2d, false);

		r_result->shape = shape_index;
		r_result->collider_id = collision_object_2d->get_instance_id();
		r_result->rid = collision_object_2d->get_rid();

		if (r_result->collider_id.is_valid()) {
			r_result->collider = Box2DSpace2D::_get_object_instance_hack(r_result->collider_id);
		}

		return true;
	}

	return false;
}

bool Box2DDirectSpaceState2D::_cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *p_closest_safe, float *p_closest_unsafe) {
	Box2DShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	box2d::ShapeHandle shape_handle = shape->get_box2d_shape();
	ERR_FAIL_COND_V(!box2d::is_handle_valid(shape_handle), false);

	b2Vec2 box2d_motion = { motion.x, motion.y };
	box2d::ShapeInfo shape_info = box2d::shape_info_from_body_shape(shape_handle, Transform2D(), transform);

	box2d::QueryExcludedInfo query_excluded_info = box2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;
	real_t hit = box2d::shape_casting(space->handle, box2d_motion, shape_info, collide_with_bodies, collide_with_areas, Box2DSpace2D::_is_handle_excluded_callback, &query_excluded_info, margin).toi;
	*p_closest_safe = hit;
	*p_closest_unsafe = hit;
	return true;
}

bool Box2DDirectSpaceState2D::_collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) {
	Box2DShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	box2d::ShapeHandle shape_handle = shape->get_box2d_shape();
	ERR_FAIL_COND_V(!box2d::is_handle_valid(shape_handle), false);

	b2Vec2 box2d_motion{ motion.x, motion.y };

	Vector2 *results_out = static_cast<Vector2 *>(results);
	box2d::ShapeInfo shape_info = box2d::shape_info_from_body_shape(shape_handle, Transform2D(), transform);
	box2d::QueryExcludedInfo query_excluded_info = box2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;
	query_excluded_info.query_exclude = (b2Fixture **)memalloc((max_results) * sizeof(b2Fixture *));
	query_excluded_info.query_exclude_size = 0;

	int cpt = 0;
	int array_idx = 0;
	do {
		box2d::ShapeCastResult result = box2d::shape_casting(space->handle, box2d_motion, shape_info, collide_with_bodies, collide_with_areas, Box2DSpace2D::_is_handle_excluded_callback, &query_excluded_info, margin);
		if (!result.collided) {
			break;
		}
		(*result_count)++;
		query_excluded_info.query_exclude[query_excluded_info.query_exclude_size++] = result.collider;

		results_out[array_idx++] = Vector2(result.witness1.x, result.witness1.y);
		results_out[array_idx++] = Vector2(result.witness2.x, result.witness2.y);

		cpt++;
	} while (cpt < max_results);

	return array_idx > 0;
}

int Box2DDirectSpaceState2D::_intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *r_results, int32_t p_result_max) {
	Box2DShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	box2d::ShapeHandle shape_handle = shape->get_box2d_shape();
	ERR_FAIL_COND_V(!box2d::is_handle_valid(shape_handle), false);

	b2Vec2 box2d_motion{ motion.x, motion.y };
	box2d::ShapeInfo shape_info = box2d::shape_info_from_body_shape(shape_handle, Transform2D(), transform);

	box2d::QueryExcludedInfo query_excluded_info = box2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;
	query_excluded_info.query_exclude = (b2Fixture **)memalloc((p_result_max) * sizeof(b2Fixture *));
	query_excluded_info.query_exclude_size = 0;

	int cpt = 0;
	do {
		box2d::ShapeCastResult result = box2d::shape_casting(space->handle, box2d_motion, shape_info, collide_with_bodies, collide_with_areas, Box2DSpace2D::_is_handle_excluded_callback, &query_excluded_info, margin);
		if (!result.collided) {
			break;
		}
		query_excluded_info.query_exclude[query_excluded_info.query_exclude_size++] = result.collider;

		ERR_CONTINUE_MSG(!box2d::is_user_data_valid(result.user_data), "Invalid user data");
		uint32_t shape_index = 0;
		Box2DCollisionObject2D *collision_object_2d = Box2DCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
		ERR_CONTINUE_MSG(!collision_object_2d, "Invalid collision object");

		PhysicsServer2DExtensionShapeResult &result_out = r_results[cpt];

		result_out.shape = shape_index;
		result_out.rid = collision_object_2d->get_rid();
		result_out.collider_id = collision_object_2d->get_instance_id();

		if (result_out.collider_id.is_valid()) {
			result_out.collider = Box2DSpace2D::_get_object_instance_hack(result_out.collider_id);
		}

		cpt++;

	} while (cpt < p_result_max);

	return cpt;
}

bool Box2DDirectSpaceState2D::_rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *r_info) {
	Box2DShape2D *shape = space->get_shape_from_rid(shape_rid);
	ERR_FAIL_COND_V(!shape, false);
	box2d::ShapeHandle shape_handle = shape->get_box2d_shape();
	ERR_FAIL_COND_V(!box2d::is_handle_valid(shape_handle), false);

	b2Vec2 box2d_motion{ motion.x, motion.y };
	box2d::ShapeInfo shape_info = box2d::shape_info_from_body_shape(shape_handle, Transform2D(), transform);
	box2d::QueryExcludedInfo query_excluded_info = box2d::default_query_excluded_info();
	query_excluded_info.query_collision_layer_mask = collision_mask;

	box2d::ShapeCastResult result = box2d::shape_casting(space->handle, box2d_motion, shape_info, collide_with_bodies, collide_with_areas, Box2DSpace2D::_is_handle_excluded_callback, &query_excluded_info, margin);
	if (!result.collided) {
		return false;
	}
	uint32_t shape_index = 0;
	Box2DCollisionObject2D *collision_object_2d = Box2DCollisionObject2D::get_collider_user_data(result.user_data, shape_index);
	ERR_FAIL_COND_V(!collision_object_2d, false);
	r_info->collider_id = collision_object_2d->get_instance_id();
	if (collision_object_2d->get_type() == Box2DCollisionObject2D::Type::TYPE_BODY) {
		const Box2DBody2D *body = static_cast<const Box2DBody2D *>(collision_object_2d);
		Vector2 rel_vec = r_info->point - (body->get_transform().get_origin() + body->get_center_of_mass());
		r_info->linear_velocity = Vector2(-body->get_angular_velocity() * rel_vec.y, body->get_angular_velocity() * rel_vec.x) + body->get_linear_velocity();

	} else {
		r_info->linear_velocity = Vector2();
	}
	r_info->normal = Vector2(result.normal1.x, result.normal1.y);
	r_info->rid = collision_object_2d->get_rid();
	r_info->shape = shape_index;
	return true;
}
