#include "box2d_shape.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

/* CIRCLE SHAPE */

void Box2DShapeCircle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::FLOAT);
	radius = p_data;
	configured = true;
}

Variant Box2DShapeCircle::get_data() const {
	return radius;
}

b2Shape *Box2DShapeCircle::get_transformed_b2Shape(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2CircleShape *shape = memnew(b2CircleShape);
	godot_to_box2d(radius, shape->m_radius);
	godot_to_box2d(p_transform.get_origin(), shape->m_p);
	return shape;
}

Box2DShapeCircle::Box2DShapeCircle() {
}

Box2DShapeCircle::~Box2DShapeCircle() {
}

/* RECTANGLE SHAPE */

void Box2DShapeRectangle::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::VECTOR2);
	half_extents = p_data;
	configured = true;
}

Variant Box2DShapeRectangle::get_data() const {
	return half_extents;
}

b2Shape *Box2DShapeRectangle::get_transformed_b2Shape(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 box2d_half_extents;
	godot_to_box2d(half_extents, box2d_half_extents);
	b2Vec2 box2d_origin;
	godot_to_box2d(p_transform.get_origin(), box2d_origin);
	shape->SetAsBox(box2d_half_extents.x, box2d_half_extents.y, box2d_origin, p_transform.get_rotation());
	return shape;
}

Box2DShapeRectangle::Box2DShapeRectangle() {
}

Box2DShapeRectangle::~Box2DShapeRectangle() {
}

/* CAPSULE SHAPE */

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
	configured = true;
}

Variant Box2DShapeCapsule::get_data() const {
	return Vector2(height, radius);
}

int Box2DShapeCapsule::get_b2Shape_count() {
	// TODO: Better handle the degenerate case when the capsule is a sphere.
	return 3;
}

b2Shape *Box2DShapeCapsule::get_transformed_b2Shape(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX_V(p_index, 3, nullptr);
	if (p_index == 0 || p_index == 1) {
		b2CircleShape *shape = memnew(b2CircleShape);
		godot_to_box2d(radius, shape->m_radius);
		real_t circle_height = (height * 0.5 - radius) * (p_index == 0 ? 1.0 : -1.0);
		godot_to_box2d(p_transform.xform(Vector2(0, circle_height)), shape->m_p);
		return shape;
	} else {
		b2PolygonShape *shape = memnew(b2PolygonShape);
		Vector2 half_extents(radius, height * 0.5 - radius);
		b2Vec2 box2d_half_extents;
		godot_to_box2d(half_extents, box2d_half_extents);
		b2Vec2 box2d_origin;
		godot_to_box2d(p_transform.get_origin(), box2d_origin);
		shape->SetAsBox(box2d_half_extents.x, box2d_half_extents.y, box2d_origin, p_transform.get_rotation());
		return shape;
	}
	return nullptr; // This line is never reached, but it silences the compiler warning.
}

Box2DShapeCapsule::Box2DShapeCapsule() {
}

Box2DShapeCapsule::~Box2DShapeCapsule() {
}

/* CONVEX POLYGON SHAPE */

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

b2Shape *Box2DShapeConvexPolygon::get_transformed_b2Shape(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2PolygonShape *shape = memnew(b2PolygonShape);
	b2Vec2 *box2d_points = new b2Vec2[points.size()];
	for (int i = 0; i < points.size(); i++) {
		godot_to_box2d(p_transform.xform(points[i]), box2d_points[i]);
	}
	shape->Set(box2d_points, (int)points.size());
	delete[] box2d_points;
	return shape;
}

Box2DShapeConvexPolygon::Box2DShapeConvexPolygon() {
}

Box2DShapeConvexPolygon::~Box2DShapeConvexPolygon() {
}

/* CONCAVE POLYGON SHAPE */

void Box2DShapeConcavePolygon::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY);
	PackedVector2Array points_array = p_data;
	points.resize(points_array.size());
	for (int i = 0; i < points_array.size(); i++) {
		points.write[i] = points_array[i];
	}
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

int Box2DShapeConcavePolygon::get_b2Shape_count() {
	// TODO: Split into chains instead of having separate edges?
	return points.size() / 2;
}

b2Shape *Box2DShapeConcavePolygon::get_transformed_b2Shape(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX_V(p_index, points.size() / 2, nullptr);
	b2EdgeShape *shape = memnew(b2EdgeShape);
	b2Vec2 box2d_endpoints[2];
	godot_to_box2d(p_transform.xform(points[2 * p_index]), box2d_endpoints[0]);
	godot_to_box2d(p_transform.xform(points[2 * p_index + 1]), box2d_endpoints[1]);
	shape->SetTwoSided(box2d_endpoints[0], box2d_endpoints[1]);
	return shape;
}

Box2DShapeConcavePolygon::Box2DShapeConcavePolygon() {
}

Box2DShapeConcavePolygon::~Box2DShapeConcavePolygon() {
}
