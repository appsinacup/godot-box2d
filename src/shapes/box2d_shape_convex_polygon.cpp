#include "box2d_shape_convex_polygon.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeConvexPolygon::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY);
	PackedVector2Array points_array = p_data;
	ERR_FAIL_COND(points_array.size() < 3);
	points.resize(points_array.size());
	for (int i = 0; i < points_array.size(); i++) {
		points.write[i] = points_array[i];
	}

	points = remove_points_that_are_too_close(points);
	ERR_FAIL_COND(points.size() < 3);
	polygons.clear();
	int cut_polygon_size = 0;
	// Cut convex into small N<=8 gons
	for (int i = 0; i < points.size(); i += cut_polygon_size) {
		// check how big we should make the polygon with remaining points
		cut_polygon_size = std::min(b2_maxPolygonVertices, points.size() - i);
		int remaining_count = points.size() - (i + cut_polygon_size);
		// if after this we have less than 4 points remaining, make a smaller polygon
		if (remaining_count <= 4 && remaining_count != 0) {
			cut_polygon_size -= 4;
		}
		Vector<Vector2> cut_polygon;
		cut_polygon.resize(cut_polygon_size);
		for (int j = 0; j < cut_polygon_size; j++) {
			cut_polygon.write[j] = points[i + j];
		}
		polygons.append(cut_polygon);
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

int Box2DShapeConvexPolygon::get_b2Shape_count(bool is_static) const {
	return polygons.size();
}

Vector<Vector2> Box2DShapeConvexPolygon::remove_points_that_are_too_close(Vector<Vector2> points) {
	int32 tempCount = 0;
	Vector<Vector2> new_points;
	// remove points that are too close
	for (int32 i = 0; i < points.size(); ++i) {
		Vector2 v = points[i];

		bool unique = true;
		for (int32 j = 0; j < tempCount; ++j) {
			if (v.distance_squared_to(new_points[j]) < ((0.5f * GODOT_LINEAR_SLOP) * (0.5f * GODOT_LINEAR_SLOP))) {
				unique = false;
				break;
			}
		}

		if (unique) {
			tempCount++;
			new_points.append(v);
		}
	}
	return new_points;
}

// based on https://github.com/briansemrau/godot_box2d/blob/5f55923fac81386e5735573e99d908d18efec6a1/scene/resources/box2d_shapes.cpp#L424
int Box2DShapeConvexPolygon::remove_bad_points(b2Vec2 *vertices, int32 count) {
	int32 n = b2Min(count, b2_maxPolygonVertices);

	// Perform welding and copy vertices into local buffer.
	b2Vec2 ps[b2_maxPolygonVertices];
	int32 tempCount = 0;
	for (int32 i = 0; i < n; ++i) {
		b2Vec2 v = vertices[i];

		bool unique = true;
		for (int32 j = 0; j < tempCount; ++j) {
			if (b2DistanceSquared(v, ps[j]) < ((0.5f * b2_linearSlop) * (0.5f * b2_linearSlop))) {
				unique = false;
				break;
			}
		}

		if (unique) {
			ps[tempCount++] = v;
		}
	}

	n = tempCount;
	if (n < 3) {
		ERR_PRINT("Polygon has too few vertices after welding " + itos(n));
		// Polygon has too few vertices
		return 0;
	}

	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	// Find the right most point on the hull
	int32 i0 = 0;
	float x0 = ps[0].x;
	for (int32 i = 1; i < n; ++i) {
		float x = ps[i].x;
		if (x > x0 || (x == x0 && ps[i].y < ps[i0].y)) {
			i0 = i;
			x0 = x;
		}
	}

	int32 hull[b2_maxPolygonVertices];
	int32 m = 0;
	int32 ih = i0;

	for (;;) {
		if (m >= b2_maxPolygonVertices) {
			ERR_PRINT("Cannot compute convex hull.");
			// cannot compute convex hull
			return 0;
		}
		hull[m] = ih;

		int32 ie = 0;
		for (int32 j = 1; j < n; ++j) {
			if (ie == ih) {
				ie = j;
				continue;
			}

			b2Vec2 r = ps[ie] - ps[hull[m]];
			b2Vec2 v = ps[j] - ps[hull[m]];
			float c = b2Cross(r, v);
			if (c < 0.0f) {
				ie = j;
			}

			// Collinearity check
			if (c == 0.0f && v.LengthSquared() > r.LengthSquared()) {
				ie = j;
			}
		}

		++m;
		ih = ie;

		if (ie == i0) {
			break;
		}
	}

	if (m < 3) {
		ERR_PRINT("Polygon is degenerate. Convex hull has point_count " + itos(m));
		// Polygon is degenerate.
		return 0;
	}
	// Copy vertices.
	for (int32 i = 0; i < m; ++i) {
		vertices[i] = ps[hull[i]];
	}
	return m;
}

b2Shape *Box2DShapeConvexPolygon::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_COND_V(p_index >= polygons.size(), nullptr);
	Vector<Vector2> polygon = polygons[p_index];
	ERR_FAIL_COND_V(polygon.size() > b2_maxPolygonVertices, nullptr);
	ERR_FAIL_COND_V(polygon.size() < 3, nullptr);
	b2Vec2 b2_points[b2_maxPolygonVertices];
	b2PolygonShape *polygon_shape = memnew(b2PolygonShape);
	for (int i = 0; i < polygon.size(); i++) {
		godot_to_box2d(p_transform.xform(polygon[i]), b2_points[i]);
	}
	int new_size = remove_bad_points(b2_points, polygon.size());
	ERR_FAIL_COND_V(new_size < 3, nullptr);
	polygon_shape->Set(b2_points, new_size);
	return polygon_shape;
}
