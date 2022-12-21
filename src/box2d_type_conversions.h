#ifndef BOX2D_TYPE_CONVERSIONS_H
#define BOX2D_TYPE_CONVERSIONS_H

#include <godot_cpp/variant/vector2.hpp>

#include <box2d/b2_math.h>

using namespace godot;

#define B_TO_G_FACTOR 50.0
#define G_TO_B_FACTOR 1.0/50.0

void godot_to_box2d(const real_t &x, float &box2d_x);
void godot_to_box2d(const Vector2 &pos, b2Vec2 &box2d_pos);

void box2d_to_godot(const float &box2d_x, real_t &x);
void box2d_to_godot(const b2Vec2 &box2d_pos, Vector2 &pos);

#endif // BOX2D_TYPE_CONVERSIONS_H
