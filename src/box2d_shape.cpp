#include "box2d_shape.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

/* RECTANGLE SHAPE */

void Box2DShapeRectangle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);
	half_extents = p_data;
	b2Vec2 box2d_half_extents;
	godot_to_box2d(half_extents, box2d_half_extents);
	static_cast<b2PolygonShape*>(shape)->SetAsBox(box2d_half_extents.x, box2d_half_extents.y);
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