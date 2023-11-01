#ifndef BOX2D_CAPSULE_SHAPE_2D_H
#define BOX2D_CAPSULE_SHAPE_2D_H

#include "box2d_shape_2d.h"

class Box2DCapsuleShape2D : public Box2DShape2D {
	real_t radius = 0.0;
	real_t height = 0.0;

public:
	_FORCE_INLINE_ const real_t &get_radius() const { return radius; }
	_FORCE_INLINE_ const real_t &get_height() const { return height; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CAPSULE; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;

protected:
	virtual box2d::Handle create_box2d_shape() const override;
};

#endif // BOX2D_CAPSULE_SHAPE_2D_H
