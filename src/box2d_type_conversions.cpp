#include "box2d_type_conversions.h"

void godot_to_box2d(const float &p_value, float &r_box2d_value) {
	r_box2d_value = godot_to_box2d(p_value);
}

void godot_to_box2d(const double &p_value, float &r_box2d_value) {
	r_box2d_value = godot_to_box2d(p_value);
}

void godot_to_box2d(const Vector2 &p_vector, b2Vec2 &r_box2d_vector) {
	r_box2d_vector.x = p_vector.x * G_TO_B_FACTOR;
	r_box2d_vector.y = p_vector.y * G_TO_B_FACTOR;
}

float godot_to_box2d(const float &p_value) {
	return p_value * G_TO_B_FACTOR;
}

float godot_to_box2d(const double &p_value) {
	return (float)(p_value * G_TO_B_FACTOR);
}

b2Vec2 godot_to_box2d(const Vector2 &p_vector) {
	return b2Vec2(p_vector.x * G_TO_B_FACTOR, p_vector.y * G_TO_B_FACTOR);
}

void box2d_to_godot(const float &p_box2d_value, float &r_value) {
	r_value = box2d_to_godot(p_box2d_value);
}

void box2d_to_godot(const float &p_box2d_value, double &r_value) {
	r_value = box2d_to_godot_d(p_box2d_value);
}

void box2d_to_godot(const b2Vec2 &p_box2d_vector, Vector2 &r_vector) {
	r_vector.x = B_TO_G_FACTOR * p_box2d_vector.x;
	r_vector.y = B_TO_G_FACTOR * p_box2d_vector.y;
}

float box2d_to_godot(const float &p_box2d_value) {
	return B_TO_G_FACTOR * p_box2d_value;
}
double box2d_to_godot_d(const float &p_box2d_value) {
	return (double)(B_TO_G_FACTOR * p_box2d_value);
}
Vector2 box2d_to_godot(const b2Vec2 &p_box2d_vector) {
	return Vector2(B_TO_G_FACTOR * p_box2d_vector.x, B_TO_G_FACTOR * p_box2d_vector.y);
}

float variant_to_number(Variant p_variant) {
	if (p_variant.get_type() == Variant::INT) {
		return (int)p_variant;
	}
	return p_variant;
}
