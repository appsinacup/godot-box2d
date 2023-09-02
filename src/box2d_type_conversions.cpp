#include "box2d_type_conversions.h"

bool is_zero(const float &p_value) {
	if (p_value < b2_epsilon && p_value > -b2_epsilon) {
		return true;
	}
	return false;
}

float ensure_non_zero(const float &p_value) {
	if (p_value < b2_epsilon && p_value > -b2_epsilon) {
		return b2_epsilon;
	}
	return p_value;
}

float godot_to_box2d(const float &p_value) {
	return p_value * (1.0f/Box2DProjectSettings::get_scaling_factor());
}

b2Vec2 godot_to_box2d(const Vector2 &p_vector) {
	return b2Vec2(godot_to_box2d(p_vector.x), godot_to_box2d(p_vector.y));
}

float box2d_to_godot(const float &p_box2d_value) {
	return Box2DProjectSettings::get_scaling_factor() * p_box2d_value;
}
Vector2 box2d_to_godot(const b2Vec2 &p_box2d_vector) {
	return Vector2(box2d_to_godot(p_box2d_vector.x), box2d_to_godot(p_box2d_vector.y));
}

float variant_to_number(Variant p_variant) {
	if (p_variant.get_type() == Variant::INT) {
		return (int)p_variant;
	}
	return p_variant;
}
