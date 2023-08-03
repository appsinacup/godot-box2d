#include "box2d_shape_concave_polygon.h"
#include "../box2d_type_conversions.h"

#include "box2d_shape_convex_polygon.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_polygon_shape.h>

constexpr float CHAIN_SEGMENT_SQUARE_SIZE = 0.5f;

void Box2DShapeConcavePolygon::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY);
	PackedVector2Array points_array = p_data;
	points.resize(points_array.size());
	for (int i = 0; i < points_array.size(); i++) {
		points.write[i] = points_array[i];
	}
	points = Box2DShapeConvexPolygon::make_sure_polygon_is_counterclockwise(points);
	points = Box2DShapeConvexPolygon::remove_points_that_are_too_close(points);
	configured = true;
	// update all existing shapes
	reconfigure_all_b2Shapes();
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
b2Shape *Box2DShapeConcavePolygon::get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	// make a chain shape if it's static
	if (shape_info.is_static) {
		ERR_FAIL_INDEX_V(shape_info.index, 1, nullptr);
		b2ChainShape *shape = memnew(b2ChainShape);
		created_shapes.append(shape);
		if (body) {
			shape_body_map[shape] = body;
		}
		ERR_FAIL_INDEX_V(shape_info.index, 1, nullptr);
		b2Vec2 *box2d_points = new b2Vec2[points.size()];
		for (int i = 0; i < points.size(); i++) {
			box2d_points[i] = godot_to_box2d(shape_info.transform.xform(points[i]));
		}
		int points_count = points.size();
		ERR_FAIL_COND_V(points_count < 3, nullptr);
		shape->CreateLoop(box2d_points, points_count);
		delete[] box2d_points;
		return shape;
	}
	ERR_FAIL_COND_V(shape_info.index > points.size(), nullptr);
	// if not make multiple small squares the size of a line
	b2PolygonShape *shape = memnew(b2PolygonShape);
	created_shapes.append(shape);
	if (body) {
		shape_body_map[shape] = body;
	}
	b2Vec2 *box2d_points = new b2Vec2[4];
	Vector2 a = points[shape_info.index];
	Vector2 b = points[(shape_info.index + 1) % points.size()];
	Vector2 dir = (a - b).normalized();
	Vector2 right(dir.y, -dir.x);
	box2d_points[0] = godot_to_box2d(shape_info.transform.xform(a - right * CHAIN_SEGMENT_SQUARE_SIZE));
	box2d_points[1] = godot_to_box2d(shape_info.transform.xform(a + right * CHAIN_SEGMENT_SQUARE_SIZE));
	box2d_points[2] = godot_to_box2d(shape_info.transform.xform(b - right * CHAIN_SEGMENT_SQUARE_SIZE));
	box2d_points[3] = godot_to_box2d(shape_info.transform.xform(b + right * CHAIN_SEGMENT_SQUARE_SIZE));
	shape->Set(box2d_points, 4);
	delete[] box2d_points;
	return shape;
}
