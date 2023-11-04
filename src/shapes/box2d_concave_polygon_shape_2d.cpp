#include "box2d_concave_polygon_shape_2d.h"

b2Shape* Box2DConcavePolygonShape2D::create_box2d_shape() const {
	int point_count = points.size();
	ERR_FAIL_COND_V(point_count < 3, box2d::invalid_shape_handle());
	b2Vec2 *box2d_points = (b2Vec2 *)alloca(point_count * sizeof(b2Vec2));
	for (int i = 0; i < point_count; i++) {
		box2d_points[i] = b2Vec2{ (points[i].x), (points[i].y) };
	}
	return box2d::shape_create_convave_polyline(box2d_points, point_count);
}

void Box2DConcavePolygonShape2D::set_data(const Variant &p_data) {
#ifdef REAL_T_IS_DOUBLE
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT64_ARRAY);
#else
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT32_ARRAY);
#endif

	Rect2 aabb;

	if (p_data.get_type() == Variant::PACKED_VECTOR2_ARRAY) {
		PackedVector2Array p2arr = p_data;
		int len = p2arr.size();
		ERR_FAIL_COND(len % 2);

		segments.clear();
		points.clear();

		if (len == 0) {
			configure(aabb);
			return;
		}

		const Vector2 *arr = p2arr.ptr();

		HashMap<Point2, int> pointmap;
		for (int i = 0; i < len; i += 2) {
			Point2 p1 = arr[i];
			Point2 p2 = arr[i + 1];
			int idx_p1, idx_p2;

			if (pointmap.has(p1)) {
				idx_p1 = pointmap[p1];
			} else {
				idx_p1 = pointmap.size();
				pointmap[p1] = idx_p1;
			}

			if (pointmap.has(p2)) {
				idx_p2 = pointmap[p2];
			} else {
				idx_p2 = pointmap.size();
				pointmap[p2] = idx_p2;
			}

			Segment s;
			s.points[0] = idx_p1;
			s.points[1] = idx_p2;
			segments.push_back(s);
		}

		points.resize(pointmap.size());
		aabb.position = pointmap.begin()->key;
		for (const KeyValue<Point2, int> &E : pointmap) {
			aabb.expand_to(E.key);
			points[E.value] = E.key;
		}
	} else {
		//dictionary with arrays
	}

	configure(aabb);
}

Variant Box2DConcavePolygonShape2D::get_data() const {
	PackedVector2Array rsegments;
	int len = segments.size();
	if (len == 0) {
		return rsegments;
	}

	rsegments.resize(len * 2);
	Vector2 *w = rsegments.ptrw();
	for (int i = 0; i < len; i++) {
		w[(i << 1) + 0] = points[segments[i].points[0]];
		w[(i << 1) + 1] = points[segments[i].points[1]];
	}

	return rsegments;
}
