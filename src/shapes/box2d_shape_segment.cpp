#include "box2d_shape_segment.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

constexpr float SEGMENT_SIZE = 0.5f;

void Box2DShapeSegment::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::RECT2);
	Rect2 rect = p_data;
	a = rect.get_position();
	b = rect.get_size();
	half_extents = Vector2((a - b).length(), GODOT_LINEAR_SLOP);
	half_extents.x = ensure_non_zero(half_extents.x);
	half_extents.y = ensure_non_zero(half_extents.y);
	configured = true;
	// update all existing shapes
	reconfigure_all_b2Shapes();
}

Variant Box2DShapeSegment::get_data() const {
	return Rect2(a, b);
}

b2Shape *Box2DShapeSegment::get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	ERR_FAIL_INDEX_V(shape_info.index, 1, nullptr);
	// make a line if it's static
	if (shape_info.is_static) {
		b2EdgeShape *shape = memnew(b2EdgeShape);
		created_shapes.append(shape);
		if (body) {
			shape_body_map[shape] = body;
		}
		b2Vec2 edge_endpoints[2];
		edge_endpoints[0] = godot_to_box2d(shape_info.transform.xform(a));
		edge_endpoints[1] = godot_to_box2d(shape_info.transform.xform(b));
		if (shape_info.one_way) {
			b2Vec2 dirV0 = edge_endpoints[0] - edge_endpoints[1];
			shape->SetOneSided(edge_endpoints[1] + dirV0, edge_endpoints[0], edge_endpoints[1], edge_endpoints[0] - dirV0);
		} else {
			shape->SetTwoSided(edge_endpoints[0], edge_endpoints[1]);
		}
		return shape;
	}
	// make a square if not
	b2PolygonShape *shape = memnew(b2PolygonShape);
	created_shapes.append(shape);
	if (body) {
		shape_body_map[shape] = body;
	}
	b2Vec2 *box2d_points = new b2Vec2[4];
	Vector2 dir = (a - b).normalized();
	Vector2 right(dir.y, -dir.x);
	box2d_points[0] = godot_to_box2d(shape_info.transform.xform(a - right * SEGMENT_SIZE));
	box2d_points[1] = godot_to_box2d(shape_info.transform.xform(a + right * SEGMENT_SIZE));
	box2d_points[2] = godot_to_box2d(shape_info.transform.xform(b - right * SEGMENT_SIZE));
	box2d_points[3] = godot_to_box2d(shape_info.transform.xform(b + right * SEGMENT_SIZE));
	shape->Set(box2d_points, 4);
	delete[] box2d_points;
	return shape;
}
