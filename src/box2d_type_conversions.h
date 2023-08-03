#pragma once

#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <box2d/b2_math.h>

using namespace godot;

constexpr float BOX2D_TO_GODOT_FACTOR = 100.0f;
constexpr float GODOT_TO_BOX2D_FACTOR = 1.0f / 100.0f;
constexpr float GODOT_LINEAR_SLOP = b2_linearSlop * BOX2D_TO_GODOT_FACTOR;

bool is_zero(const float &p_value);
float ensure_non_zero(const float &p_value);

float godot_to_box2d(const float &p_value, float scale_factor = GODOT_TO_BOX2D_FACTOR);
b2Vec2 godot_to_box2d(const Vector2 &p_vector, float scale_factor = GODOT_TO_BOX2D_FACTOR);

float box2d_to_godot(const float &p_box2d_value, float scale_factor = BOX2D_TO_GODOT_FACTOR);
Vector2 box2d_to_godot(const b2Vec2 &p_box2d_vector, float scale_factor = BOX2D_TO_GODOT_FACTOR);

float variant_to_number(Variant p_variant);
