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

float godot_to_box2d(const float &p_value, float scale_factor) {
	return p_value * scale_factor;
}

b2Vec2 godot_to_box2d(const Vector2 &p_vector, float scale_factor) {
	return b2Vec2(godot_to_box2d(p_vector.x, scale_factor), godot_to_box2d(p_vector.y, scale_factor));
}

float box2d_to_godot(const float &p_box2d_value, float scale_factor) {
	return scale_factor * p_box2d_value;
}
Vector2 box2d_to_godot(const b2Vec2 &p_box2d_vector, float scale_factor) {
	return Vector2(box2d_to_godot(p_box2d_vector.x, scale_factor), box2d_to_godot(p_box2d_vector.y, scale_factor));
}

float variant_to_number(Variant p_variant) {
	if (p_variant.get_type() == Variant::INT) {
		return (int)p_variant;
	}
	return p_variant;
}
