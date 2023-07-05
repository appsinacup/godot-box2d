#include "box2d_shape_capsule.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeCapsule::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY && p_data.get_type() != Variant::VECTOR2);
	if (p_data.get_type() == Variant::ARRAY) {
		Array arr = p_data;
		ERR_FAIL_COND(arr.size() != 2);
		height = arr[0];
		radius = arr[1];
	} else {
		Point2 p = p_data;
		radius = p.x;
		height = p.y;
	}
	if (radius < GODOT_LINEAR_SLOP) {
		radius = GODOT_LINEAR_SLOP;
	}
	if (height < GODOT_LINEAR_SLOP) {
		height = GODOT_LINEAR_SLOP;
	}
	configured = true;
}

Variant Box2DShapeCapsule::get_data() const {
	return Vector2(height, radius);
}

int Box2DShapeCapsule::get_b2Shape_count(bool is_static) const {
	// TODO: Better handle the degenerate case when the capsule is a sphere.
	return 3;
}

b2Shape *Box2DShapeCapsule::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_INDEX_V(p_index, 3, nullptr);
	if (p_index == 0 || p_index == 1) {
		b2CircleShape *shape = memnew(b2CircleShape);
		godot_to_box2d(radius, shape->m_radius);
		real_t circle_height = (height * 0.5 - radius) * (p_index == 0 ? 1.0 : -1.0);
		if (circle_height < b2_linearSlop) {
			circle_height = b2_linearSlop;
		}
		godot_to_box2d(p_transform.xform(Vector2(0, circle_height)), shape->m_p);
		return shape;
	}
	b2PolygonShape *shape = memnew(b2PolygonShape);
	Vector2 half_extents(radius, height * 0.5 - radius);
	if (half_extents.x < GODOT_LINEAR_SLOP) {
		half_extents.x = GODOT_LINEAR_SLOP;
	}
	if (half_extents.y < GODOT_LINEAR_SLOP) {
		half_extents.y = GODOT_LINEAR_SLOP;
	}
	b2Vec2 box2d_half_extents = godot_to_box2d(half_extents);
	b2Vec2 box2d_origin = godot_to_box2d(p_transform.get_origin());
	shape->SetAsBox(box2d_half_extents.x, box2d_half_extents.y, box2d_origin, p_transform.get_rotation());
	return shape;
}
