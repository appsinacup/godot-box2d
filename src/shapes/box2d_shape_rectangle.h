#pragma once
#include "box2d_shape_convex_polygon.h"

class Box2DShapeRectangle : public Box2DShapeConvexPolygon {
protected:
	Vector2 half_extents;

public:
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	Box2DShapeRectangle() { type = PhysicsServer2D::SHAPE_RECTANGLE; }
	~Box2DShapeRectangle() {}
};
