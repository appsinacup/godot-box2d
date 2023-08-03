#include "box2d_shape_capsule.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeCapsule::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY && p_data.get_type() != Variant::VECTOR2);
	if (p_data.get_type() == Variant::ARRAY) {
		Array arr = p_data;
		ERR_FAIL_COND(arr.size() != 2);
		height = arr[0];
		radius = arr[1];
	} else {
		Point2 p = p_data;
		radius = p.x;
		height = p.y;
	}
	radius = ensure_non_zero(radius);
	height = ensure_non_zero(height);
	configured = true;
	// update all existing shapes
	reconfigure_all_b2Shapes();
}

Variant Box2DShapeCapsule::get_data() const {
	return Vector2(height, radius);
}

int Box2DShapeCapsule::get_b2Shape_count(bool is_static) const {
	return 3;
}
b2Shape *Box2DShapeCapsule::get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	ERR_FAIL_INDEX_V(shape_info.index, 3, nullptr);
	Vector2 scale = shape_info.transform.get_scale();
	if (scale.x != scale.y) {
		ERR_PRINT("Capsules don't support non uniform scale.");
	}
	float radius_scaled = godot_to_box2d(radius * scale.x);
	if (shape_info.index == 0 || shape_info.index == 1) {
		b2CircleShape *shape = memnew(b2CircleShape);
		created_shapes.append(shape);
		if (body) {
			shape_body_map[shape] = body;
		}
		shape->m_radius = radius_scaled;
		real_t circle_height = (height * 0.5f - radius) * (shape_info.index == 0 ? 1.0f : -1.0f);
		circle_height = ensure_non_zero(circle_height);
		shape->m_p = godot_to_box2d(shape_info.transform.xform(Vector2(0.0f, circle_height)));
		return shape;
	}
	b2PolygonShape *shape = memnew(b2PolygonShape);
	created_shapes.append(shape);
	if (body) {
		shape_body_map[shape] = body;
	}
	Vector2 half_extents(radius, height * 0.5 - radius);
	half_extents.x = ensure_non_zero(half_extents.x);
	half_extents.y = ensure_non_zero(half_extents.y);

	b2Vec2 box2d_half_extents = godot_to_box2d(half_extents);
	b2Vec2 *box2d_points = new b2Vec2[4];
	box2d_points[0] = godot_to_box2d(shape_info.transform.xform(Vector2(-half_extents.x, -half_extents.y)));
	box2d_points[1] = godot_to_box2d(shape_info.transform.xform(Vector2(-half_extents.x, half_extents.y)));
	box2d_points[2] = godot_to_box2d(shape_info.transform.xform(Vector2(half_extents.x, half_extents.y)));
	box2d_points[3] = godot_to_box2d(shape_info.transform.xform(Vector2(half_extents.x, -half_extents.y)));
	shape->Set(box2d_points, 4);
	delete[] box2d_points;
	return shape;
}
