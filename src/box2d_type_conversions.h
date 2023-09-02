#pragma once

#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include "servers/box2d_project_settings.h"
#include <box2d/b2_math.h>

using namespace godot;

bool is_zero(const float &p_value);
float ensure_non_zero(const float &p_value);

float godot_to_box2d(const float &p_value);
b2Vec2 godot_to_box2d(const Vector2 &p_vector);

float box2d_to_godot(const float &p_box2d_value);
Vector2 box2d_to_godot(const b2Vec2 &p_box2d_vector);

float variant_to_number(Variant p_variant);
