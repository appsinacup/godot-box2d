#include "box2d_shape.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_circle_shape.h>
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

b2Shape* Box2DShapeCircle::get_transformed_b2Shape(const Transform2D &p_transform) {
	b2CircleShape *shape = new b2CircleShape;
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

b2Shape* Box2DShapeRectangle::get_transformed_b2Shape(const Transform2D &p_transform) {
	b2PolygonShape *shape = new b2PolygonShape;
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

b2Shape* Box2DShapeConvexPolygon::get_transformed_b2Shape(const Transform2D &p_transform) {
	b2PolygonShape *shape = new b2PolygonShape;
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
