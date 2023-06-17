#include "box2d_shape_rectangle.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_polygon_shape.h>

void Box2DShapeRectangle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);
	half_extents = p_data;
	if (half_extents.x < GODOT_LINEAR_SLOP) {
		half_extents.x = GODOT_LINEAR_SLOP;
	}
	if (half_extents.y < GODOT_LINEAR_SLOP) {
		half_extents.y = GODOT_LINEAR_SLOP;
	}
	configured = true;
}

Variant Box2DShapeRectangle::get_data() const {
	return half_extents;
}

b2Shape *Box2DShapeRectangle::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 box2d_half_extents = godot_to_box2d(half_extents);
	b2Vec2 *box2d_points = new b2Vec2[4];
	godot_to_box2d(p_transform.xform(Vector2(-half_extents.x, -half_extents.y)), box2d_points[0]);
	godot_to_box2d(p_transform.xform(Vector2(-half_extents.x, half_extents.y)), box2d_points[1]);
	godot_to_box2d(p_transform.xform(Vector2(half_extents.x, half_extents.y)), box2d_points[2]);
	godot_to_box2d(p_transform.xform(Vector2(half_extents.x, -half_extents.y)), box2d_points[3]);
	shape->Set(box2d_points, 4);
	delete[] box2d_points;
	return shape;
}
