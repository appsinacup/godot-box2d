#include "box2d_segment_shape_2d.h"

b2Shape *Box2DSegmentShape2D::create_box2d_shape() const {
	Vector2 direction = b - a;
	direction.normalize();

	Vector2 perpendicular = Vector2(-direction.y, direction.x);
	double height = 1;

	Vector2 p1 = a + perpendicular * height;
	Vector2 p2 = a - perpendicular * height;
	Vector2 p3 = b + perpendicular * height;
	Vector2 p4 = b - perpendicular * height;

	b2Vec2 box2d_points[4];
	box2d_points[0] = b2Vec2{ p1.x, p1.y };
	box2d_points[1] = b2Vec2{ p2.x, p2.y };
	box2d_points[2] = b2Vec2{ p3.x, p3.y };
	box2d_points[3] = b2Vec2{ p4.x, p4.y };

	return box2d::shape_create_convex_polyline(box2d_points, 4);
}

void Box2DSegmentShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::RECT2);

	Rect2 r = p_data;
	a = r.position;
	b = r.size;
	n = (b - a).orthogonal();

	Rect2 aabb;
	aabb.position = a;
	aabb.expand_to(b);
	if (aabb.size.x == 0) {
		aabb.size.x = 0.001;
	}
	if (aabb.size.y == 0) {
		aabb.size.y = 0.001;
	}
	configure(aabb);
}

Variant Box2DSegmentShape2D::get_data() const {
	Rect2 r;
	r.position = a;
	r.size = b;
	return r;
}

real_t Box2DSegmentShape2D::get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const {
	return p_mass * ((a * p_scale).distance_squared_to(b * p_scale)) / 12.0;
}
