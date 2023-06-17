#include "box2d_shape_convex_polygon.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeConvexPolygon::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY);
	PackedVector2Array points_array = p_data;
	points.resize(points_array.size());
	for (int i = 0; i < points_array.size(); i++) {
		points.write[i] = points_array[i];
	}
	configured = true;
}

Variant Box2DShapeConvexPolygon::get_data() const {
	Array points_array;
	points_array.resize(points.size());
	for (int i = 0; i < points.size(); i++) {
		points_array[i] = points[i];
	}
	return points_array;
}

b2Shape *Box2DShapeConvexPolygon::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	if (points.size() >= b2_maxPolygonVertices) {
		// create a chain loop
		b2ChainShape *shape = memnew(b2ChainShape);
		b2Vec2 *box2d_points = new b2Vec2[points.size()];
		for (int i = 0; i < points.size(); i++) {
			godot_to_box2d(p_transform.xform(points[i]), box2d_points[i]);
		}
		shape->CreateLoop(box2d_points, (int)points.size());
		delete[] box2d_points;
		return shape;
	}
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 *box2d_points = new b2Vec2[points.size()];
	for (int i = 0; i < points.size(); i++) {
		godot_to_box2d(p_transform.xform(points[i]), box2d_points[i]);
	}
	shape->Set(box2d_points, (int)points.size());
	delete[] box2d_points;
	return shape;
}
