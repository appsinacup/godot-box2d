#include "box2d_convex_polygon_shape_2d.h"

box2d::ShapeHandle Box2DConvexPolygonShape2D::create_box2d_shape() const {
	ERR_FAIL_COND_V(point_count < 3, box2d::invalid_shape_handle());
	b2Vec2 *box2d_points = (b2Vec2 *)alloca(point_count * sizeof(b2Vec2));
	for (int i = 0; i < point_count; i++) {
		box2d_points[i] = b2Vec2{ (points[i].pos.x), (points[i].pos.y) };
	}
	return box2d::shape_create_convex_polyline(box2d_points, point_count);
}

void Box2DConvexPolygonShape2D::set_data(const Variant &p_data) {
#ifdef REAL_T_IS_DOUBLE
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT64_ARRAY);
#else
	ERR_FAIL_COND(p_data.get_type() != Variant::PACKED_VECTOR2_ARRAY && p_data.get_type() != Variant::PACKED_FLOAT32_ARRAY);
#endif

	if (points) {
		memdelete_arr(points);
	}
	points = nullptr;
	point_count = 0;

	if (p_data.get_type() == Variant::PACKED_VECTOR2_ARRAY) {
		PackedVector2Array arr = p_data;
		ERR_FAIL_COND(arr.size() == 0);
		point_count = arr.size();
		points = memnew_arr(Point, point_count);
		const Vector2 *r = arr.ptr();

		for (int i = 0; i < point_count; i++) {
			points[i].pos = r[i];
		}

		for (int i = 0; i < point_count; i++) {
			Vector2 p = points[i].pos;
			Vector2 pn = points[(i + 1) % point_count].pos;
			points[i].normal = (pn - p).orthogonal().normalized();
		}
	} else {
#ifdef REAL_T_IS_DOUBLE
		PackedFloat64Array dvr = p_data;
#else
		PackedFloat32Array dvr = p_data;
#endif
		point_count = dvr.size() / 4;
		ERR_FAIL_COND(point_count == 0);

		points = memnew_arr(Point, point_count);
		const real_t *r = dvr.ptr();

		for (int i = 0; i < point_count; i++) {
			int idx = i << 2;
			points[i].pos.x = r[idx + 0];
			points[i].pos.y = r[idx + 1];
			points[i].normal.x = r[idx + 2];
			points[i].normal.y = r[idx + 3];
		}
	}

	ERR_FAIL_COND(point_count == 0);
	Rect2 aabb;
	aabb.position = points[0].pos;
	for (int i = 1; i < point_count; i++) {
		aabb.expand_to(points[i].pos);
	}

	configure(aabb);
}

Variant Box2DConvexPolygonShape2D::get_data() const {
	PackedVector2Array dvr;

	dvr.resize(point_count);

	for (int i = 0; i < point_count; i++) {
		dvr.set(i, points[i].pos);
	}

	return dvr;
}

real_t Box2DConvexPolygonShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	ERR_FAIL_COND_V_MSG(point_count == 0, 0, "Convex polygon shape has no points.");
	Rect2 aabb_new;
	aabb_new.position = points[0].pos * p_scale;
	for (int i = 0; i < point_count; i++) {
		aabb_new.expand_to(points[i].pos * p_scale);
	}

	return p_mass * aabb_new.size.dot(aabb_new.size) / 12.0;
}

Box2DConvexPolygonShape2D::~Box2DConvexPolygonShape2D() {
	if (points) {
		memdelete_arr(points);
	}
}
