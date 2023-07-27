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
	// update all existing shapes
	configure_all_b2Shapes();
}

Variant Box2DShapeCircle::get_data() const {
	return radius;
}
b2Shape *Box2DShapeCircle::get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	b2CircleShape *shape = memnew(b2CircleShape);
	created_shapes.append(shape);
	if (body) {
		shape_body_map[shape] = body;
	}
	ERR_FAIL_INDEX_V(shape_info.index, 1, nullptr);
	Vector2 scale = shape_info.transform.get_scale();
	if (scale.x != scale.y) {
		ERR_PRINT("Circles don't support non uniform scale.");
	}
	godot_to_box2d(radius * scale.x, shape->m_radius);
	if (shape->m_radius < b2_linearSlop) {
		shape->m_radius = b2_linearSlop;
	}
	godot_to_box2d(shape_info.transform.get_origin(), shape->m_p);
	return shape;
}
