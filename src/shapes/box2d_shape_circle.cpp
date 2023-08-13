#include "box2d_shape_circle.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>

void Box2DShapeCircle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::FLOAT && p_data.get_type() != Variant::INT);
	radius = variant_to_number(p_data);
	radius = ensure_non_zero(radius);
	configured = true;
	// update all existing shapes
	reconfigure_all_b2Shapes();
}

Variant Box2DShapeCircle::get_data() const {
	return radius;
}
b2Shape *Box2DShapeCircle::_get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	ERR_FAIL_INDEX_V(shape_info.index, 1, nullptr);
	b2CircleShape *shape = memnew(b2CircleShape);
	Vector2 scale = shape_info.transform.get_scale();
	if (fabs(scale.x - scale.y) > b2_epsilon) {
		ERR_PRINT("Circles don't support non uniform scale.");
	}
	shape->m_radius = godot_to_box2d(radius * scale.x);
	shape->m_radius = ensure_non_zero(shape->m_radius);
	shape->m_p = godot_to_box2d(shape_info.transform.get_origin());
	return shape;
}
