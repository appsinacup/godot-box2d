#ifndef BOX2D_RECTANGLE_SHAPE_2D_H
#define BOX2D_RECTANGLE_SHAPE_2D_H

#include "box2d_shape_2d.h"

class Box2DRectangleShape2D : public Box2DShape2D {
	Vector2 half_extents;

protected:
	virtual box2d::Handle create_box2d_shape() const override;

public:
	_FORCE_INLINE_ const Vector2 &get_half_extents() const { return half_extents; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_RECTANGLE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // BOX2D_RECTANGLE_SHAPE_2D_H
