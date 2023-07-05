#pragma once
#include "box2d_shape.h"
#include "box2d_shape_segment.h"

class Box2DShapeSeparationRay : public Box2DShapeSegment {
public:
	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	Box2DShapeSeparationRay() { type = PhysicsServer2D::SHAPE_SEPARATION_RAY; }
	~Box2DShapeSeparationRay() {}
};
