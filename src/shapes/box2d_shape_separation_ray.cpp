#include "box2d_shape_separation_ray.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeSeparationRay::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	Dictionary dict = p_data;
	ERR_FAIL_COND(dict.size() != 2);
	ERR_FAIL_COND(!dict.has("length"));
	ERR_FAIL_COND(!dict.has("slide_on_slope"));
	float length = dict["length"];
	bool slide_on_slope = dict["slide_on_slope"];
	a = Vector2();
	b = Vector2(0, length);
	half_extents = Vector2((a - b).length(), GODOT_LINEAR_SLOP);
	if (half_extents.x < GODOT_LINEAR_SLOP) {
		half_extents.x = GODOT_LINEAR_SLOP;
	}
	configured = true;
}

Variant Box2DShapeSeparationRay::get_data() const {
	Dictionary dict;
	dict["length"] = b.x;
	dict["slide_on_slope"] = false;
	return dict;
}
