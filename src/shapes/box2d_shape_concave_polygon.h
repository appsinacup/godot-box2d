#pragma once
#include "box2d_shape.h"

class Box2DShapeConcavePolygon : public Box2DShape {
	Vector<Vector2> points;

public:
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;
	virtual int get_b2Shape_count(bool is_static) const override;
	virtual b2Shape *get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) override;

	Box2DShapeConcavePolygon() { type = PhysicsServer2D::SHAPE_CONCAVE_POLYGON; }
	~Box2DShapeConcavePolygon() {}
};
