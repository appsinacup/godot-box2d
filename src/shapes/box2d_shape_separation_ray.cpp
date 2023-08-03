#include "box2d_shape_separation_ray.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

constexpr const char *SEPARATION_RAY_LENGTH = "length";
constexpr const char *SEPARATION_RAY_SLIDE_ON_SLOPE = "slide_on_slope";

void Box2DShapeSeparationRay::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	Dictionary dict = p_data;
	ERR_FAIL_COND(dict.size() != 2);
	ERR_FAIL_COND(!dict.has(SEPARATION_RAY_LENGTH));
	ERR_FAIL_COND(!dict.has(SEPARATION_RAY_SLIDE_ON_SLOPE));
	double length = dict[SEPARATION_RAY_LENGTH];
	bool slide_on_slope = dict[SEPARATION_RAY_SLIDE_ON_SLOPE];
	a = Vector2();
	b = Vector2(0, length);
	half_extents = Vector2((a - b).length(), GODOT_LINEAR_SLOP);
	half_extents.x = ensure_non_zero(half_extents.x);
	half_extents.y = ensure_non_zero(half_extents.y);
	configured = true;
	// update all existing shapes
	reconfigure_all_b2Shapes();
}

Variant Box2DShapeSeparationRay::get_data() const {
	Dictionary dict;
	dict[SEPARATION_RAY_LENGTH] = b.x;
	dict[SEPARATION_RAY_SLIDE_ON_SLOPE] = false;
	return dict;
}
