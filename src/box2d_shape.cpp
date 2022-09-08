#include "box2d_shape.h"

#include <godot_cpp/core/memory.hpp>

/* RECTANGLE SHAPE */

void Box2DShapeRectangle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);
	half_extents = p_data;
	static_cast<b2PolygonShape*>(shape)->SetAsBox(half_extents.x, half_extents.y);
	configured = true;
}

Variant Box2DShapeRectangle::get_data() const {
	return half_extents;
}

Box2DShapeRectangle::Box2DShapeRectangle() {
	shape = memnew(b2PolygonShape);
}

Box2DShapeRectangle::~Box2DShapeRectangle() {
	memdelete(shape);
}