#pragma once
#include "box2d_shape.h"

class Box2DShapeConvexPolygon : public Box2DShape {
	Vector<Vector2> points;
	Vector<Vector<Vector2>> polygons;

public:
	static Vector<Vector2> remove_points_that_are_too_close(Vector<Vector2> points);
	static int remove_bad_points(b2Vec2 *vertices, int32 count);
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;
	virtual int get_b2Shape_count(bool is_static) const override;
	virtual b2Shape *get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) override;

	Box2DShapeConvexPolygon() { type = PhysicsServer2D::SHAPE_CONVEX_POLYGON; }
	~Box2DShapeConvexPolygon() {}
};
