#include "box2d_world_boundary_shape_2d.h"

b2Shape* Box2DWorldBoundaryShape2D::create_box2d_shape() const {
	b2Vec2 v = { normal.x, normal.y };
	return box2d::shape_create_halfspace(v);
}

void Box2DWorldBoundaryShape2D::apply_box2d_transform(b2Vec2 &position, real_t &angle) const {
	position.x += normal.x * d;
	position.y += normal.y * d;
}

void Box2DWorldBoundaryShape2D::set_data(const Variant &p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::ARRAY);
	Array arr = p_data;
	ERR_FAIL_COND(arr.size() != 2);
	normal = arr[0];
	d = arr[1];
	configure(Rect2(Vector2(-1e4, -1e4), Vector2(1e4 * 2.0, 1e4 * 2.0)));
}

Variant Box2DWorldBoundaryShape2D::get_data() const {
	Array arr;
	arr.resize(2);
	arr[0] = normal;
	arr[1] = d;
	return arr;
}
