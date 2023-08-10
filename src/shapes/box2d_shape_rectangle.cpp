#include "box2d_shape_rectangle.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_polygon_shape.h>

void Box2DShapeRectangle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);
	half_extents = p_data;
	half_extents.x = ensure_non_zero(half_extents.x);
	half_extents.y = ensure_non_zero(half_extents.y);
	configured = true;
	// update all existing shapes
	reconfigure_all_b2Shapes();
}

Variant Box2DShapeRectangle::get_data() const {
	return half_extents;
}
b2Shape *Box2DShapeRectangle::_get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	ERR_FAIL_INDEX_V(shape_info.index, 1, nullptr);
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 box2d_half_extents = godot_to_box2d(half_extents);
	b2Vec2 box2d_points[4];
	box2d_points[0] = godot_to_box2d(shape_info.transform.xform(Vector2(-half_extents.x, -half_extents.y)));
	box2d_points[1] = godot_to_box2d(shape_info.transform.xform(Vector2(-half_extents.x, half_extents.y)));
	box2d_points[2] = godot_to_box2d(shape_info.transform.xform(Vector2(half_extents.x, half_extents.y)));
	box2d_points[3] = godot_to_box2d(shape_info.transform.xform(Vector2(half_extents.x, -half_extents.y)));
	bool result = shape->Set(box2d_points, 4);
	if (!result) {
		memdelete(shape);
		ERR_FAIL_COND_V(!result, nullptr);
	}
	return shape;
}
