#include "box2d_shape_concave_polygon.h"
#include "../box2d_type_conversions.h"

#include "box2d_shape_convex_polygon.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeConcavePolygon::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY);
	PackedVector2Array points_array = p_data;
	ERR_FAIL_COND(points_array.size() < 3);
	points.resize(points_array.size());
	for (int i = 0; i < points_array.size(); i++) {
		points.write[i] = points_array[i];
	}
	points = Box2DShapeConvexPolygon::remove_points_that_are_too_close(points);
	ERR_FAIL_COND(points.size() < 3);
	configured = true;
}

Variant Box2DShapeConcavePolygon::get_data() const {
	Array points_array;
	points_array.resize(points.size());
	for (int i = 0; i < points.size(); i++) {
		points_array[i] = points[i];
	}
	return points_array;
}
int Box2DShapeConcavePolygon::get_b2Shape_count(bool is_static) const {
	if (is_static) {
		return 1;
	}
	return points.size();
}

b2Shape *Box2DShapeConcavePolygon::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	// make a chain shape if it's static
	if (is_static) {
		ERR_FAIL_INDEX_V(p_index, 1, nullptr);
		b2ChainShape *shape = memnew(b2ChainShape);
		b2Vec2 *box2d_points = new b2Vec2[points.size()];
		for (int i = 0; i < points.size(); i++) {
			godot_to_box2d(p_transform.xform(points[i]), box2d_points[i]);
		}
		int points_count = points.size();
		points_count = Box2DShapeConvexPolygon::remove_bad_points(box2d_points, points_count);
		ERR_FAIL_COND_V(points_count < 3, nullptr);
		shape->CreateChain(box2d_points, points_count, box2d_points[points.size() - 1], box2d_points[0]);
		delete[] box2d_points;
		return shape;
	}
	ERR_FAIL_COND_V(p_index > points.size(), nullptr);
	// if not make multiple small squares the size of a line
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 *box2d_points = new b2Vec2[4];
	Vector2 a = points[p_index];
	Vector2 b = points[(p_index + 1) % points.size()];
	Vector2 dir = (a - b).normalized();
	Vector2 right(dir.y, -dir.x);
	godot_to_box2d(p_transform.xform(a - right * 0.1), box2d_points[0]);
	godot_to_box2d(p_transform.xform(a + right * 0.1), box2d_points[1]);
	godot_to_box2d(p_transform.xform(b - right * 0.1), box2d_points[2]);
	godot_to_box2d(p_transform.xform(b + right * 0.1), box2d_points[3]);
	shape->Set(box2d_points, 4);
	delete[] box2d_points;
	return shape;
}
