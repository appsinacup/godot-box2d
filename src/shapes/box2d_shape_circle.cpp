#include "box2d_shape_circle.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>

void Box2DShapeCircle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::FLOAT && p_data.get_type() != Variant::INT);
	radius = variant_to_number(p_data);
	if (radius < GODOT_LINEAR_SLOP) {
		radius = GODOT_LINEAR_SLOP;
	}
	configured = true;
}

Variant Box2DShapeCircle::get_data() const {
	return radius;
}

b2Shape *Box2DShapeCircle::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2CircleShape *shape = memnew(b2CircleShape);
	Vector2 scale = p_transform.get_scale();
	if (scale.x != scale.y) {
		ERR_PRINT("Circles don't support non uniform scale.");
	}
	godot_to_box2d(radius * scale.x, shape->m_radius);
	if (shape->m_radius < b2_linearSlop) {
		shape->m_radius = b2_linearSlop;
	}
	godot_to_box2d(p_transform.get_origin(), shape->m_p);
	return shape;
}
