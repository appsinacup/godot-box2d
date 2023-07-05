#pragma once
#include "box2d_shape.h"

class Box2DShapeRectangle : public Box2DShape {
protected:
	Vector2 half_extents;

public:
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;
	virtual int get_b2Shape_count(bool is_static) const override { return 1; }
	virtual b2Shape *get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) override;

	Box2DShapeRectangle() { type = PhysicsServer2D::SHAPE_RECTANGLE; }
	~Box2DShapeRectangle() {}
};
