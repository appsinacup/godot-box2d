#pragma once

#include "box2d_shape.h"

class Box2DShapeCapsule : public Box2DShape {
	real_t height = 0.0;
	real_t radius = 0.0;

public:
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;
	virtual int get_b2Shape_count(bool is_static) const override;
	virtual b2Shape *get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) override;

	Box2DShapeCapsule() { type = PhysicsServer2D::SHAPE_CAPSULE; }
	~Box2DShapeCapsule() {}
};
