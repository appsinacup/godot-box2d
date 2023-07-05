#include "box2d_shape_world_boundary.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_polygon_shape.h>

#define WORLD_SHAPE_SIZE 100000

void Box2DShapeWorldBoundary::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY);
	Array arr = p_data;
	ERR_FAIL_COND(arr.size() != 2);
	normal = arr[0];
	distance = arr[1]; // no need to bring it to box2d here as we will do it later
	configured = true;
}

Variant Box2DShapeWorldBoundary::get_data() const {
	Array data;
	data.resize(2);
	data[0] = normal;
	data[1] = distance; // no need to bring it to godot here as we didn't cast this one
	return data;
}

b2Shape *Box2DShapeWorldBoundary::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 points[4];
	Vector2 right(normal.y, -normal.x);
	Vector2 left(-right);
	left *= WORLD_SHAPE_SIZE;
	right *= WORLD_SHAPE_SIZE;
	left = left + normal * distance;
	right = right + normal * distance;
	godot_to_box2d(p_transform.xform(left), points[0]);
	godot_to_box2d(p_transform.xform(right), points[1]);
	godot_to_box2d(p_transform.xform(right - normal * WORLD_SHAPE_SIZE), points[2]);
	godot_to_box2d(p_transform.xform(right - normal * WORLD_SHAPE_SIZE), points[3]);
	shape->Set(points, 4);
	return shape;
}
