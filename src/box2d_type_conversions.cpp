#include "box2d_type_conversions.h"

void godot_to_box2d(const float &p_value, float &r_box2d_value) {
	r_box2d_value = p_value * G_TO_B_FACTOR;
}

void godot_to_box2d(const double &p_value, float &r_box2d_value) {
	r_box2d_value = (float)(p_value * G_TO_B_FACTOR);
}

void godot_to_box2d(const Vector2 &p_vector, b2Vec2 &r_box2d_vector) {
	r_box2d_vector.x = p_vector.x * G_TO_B_FACTOR;
	r_box2d_vector.y = p_vector.y * G_TO_B_FACTOR;
}

void box2d_to_godot(const float &p_box2d_value, float &r_value) {
	r_value = B_TO_G_FACTOR * p_box2d_value;
}

void box2d_to_godot(const float &p_box2d_value, double &r_value) {
	r_value = (double)(B_TO_G_FACTOR * p_box2d_value);
}

void box2d_to_godot(const b2Vec2 &p_box2d_vector, Vector2 &r_vector) {
	r_vector.x = B_TO_G_FACTOR * p_box2d_vector.x;
	r_vector.y = B_TO_G_FACTOR * p_box2d_vector.y;
}
