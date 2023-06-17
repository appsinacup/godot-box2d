#pragma once
#include "box2d_shape.h"

class Box2DShapeSeparationRay : public Box2DShape {
	Vector2 a;
	Vector2 b;

public:
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;
	virtual int get_b2Shape_count() const override { return 1; };
	virtual b2Shape *get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) override;

	Box2DShapeSeparationRay() { type = PhysicsServer2D::SHAPE_SEPARATION_RAY; }
	~Box2DShapeSeparationRay() {}
};
