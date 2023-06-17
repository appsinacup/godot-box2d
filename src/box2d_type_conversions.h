#pragma once

#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <box2d/b2_math.h>

using namespace godot;

#define B_TO_G_FACTOR 50.0
#define G_TO_B_FACTOR 1.0 / 50.0
#define GODOT_LINEAR_SLOP b2_linearSlop *B_TO_G_FACTOR

void godot_to_box2d(const float &p_value, float &r_box2d_value);
void godot_to_box2d(const double &p_value, float &r_box2d_value);
void godot_to_box2d(const Vector2 &p_vector, b2Vec2 &r_box2d_vector);

float godot_to_box2d(const float &p_value);
float godot_to_box2d(const double &p_value);
b2Vec2 godot_to_box2d(const Vector2 &p_vector);

void box2d_to_godot(const float &p_box2d_value, float &r_value);
void box2d_to_godot(const float &p_box2d_value, double &r_value);
void box2d_to_godot(const b2Vec2 &p_box2d_vector, Vector2 &r_vector);

float box2d_to_godot(const float &p_box2d_value);
double box2d_to_godot_d(const float &p_box2d_value);
Vector2 box2d_to_godot(const b2Vec2 &p_box2d_vector);

float variant_to_number(Variant p_variant);
