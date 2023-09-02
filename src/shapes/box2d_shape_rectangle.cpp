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
	Vector<Vector2> polygon;
	polygon.resize(4);
	polygon.write[3] = Vector2(-half_extents.x, -half_extents.y);
	polygon.write[2] = Vector2(-half_extents.x, half_extents.y);
	polygon.write[1] = Vector2(half_extents.x, half_extents.y);
	polygon.write[0] = Vector2(half_extents.x, -half_extents.y);
	polygons.clear();
	polygons.append(polygon);
	// update all existing shapes
	reconfigure_all_b2Shapes();
}

Variant Box2DShapeRectangle::get_data() const {
	return half_extents;
}
