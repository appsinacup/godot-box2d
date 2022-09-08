#include "box2d_shape.h"

/* RECTANGLE SHAPE */

void Box2DShapeRectangle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);
	half_extents = p_data;
	shape.SetAsBox(half_extents.x, half_extents.y);
	configured = true;
}

Variant Box2DShapeRectangle::get_data() const {
	return half_extents;
}
