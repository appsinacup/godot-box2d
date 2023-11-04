#ifndef BOX2D_CIRCLE_SHAPE_2D_H
#define BOX2D_CIRCLE_SHAPE_2D_H

#include "box2d_shape_2d.h"

class Box2DCircleShape2D : public Box2DShape2D {
	real_t radius;

protected:
	virtual b2Shape* create_box2d_shape() const override;

public:
	_FORCE_INLINE_ const real_t &get_radius() const { return radius; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CIRCLE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // BOX2D_CIRCLE_SHAPE_2D_H
