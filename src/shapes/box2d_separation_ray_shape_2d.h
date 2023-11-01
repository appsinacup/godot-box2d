#ifndef BOX2D_SEPARATION_RAY_SHAPE_2D_H
#define BOX2D_SEPARATION_RAY_SHAPE_2D_H

#include "box2d_segment_shape_2d.h"

class Box2DSeparationRayShape2D : public Box2DSegmentShape2D {
	real_t length = 0.0;
	bool slide_on_slope = false;

protected:
	//virtual box2d::Handle create_box2d_shape() const override;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_SEPARATION_RAY; }

	//virtual void apply_box2d_transform(box2d::Vector &position, real_t &angle) const override;

	_FORCE_INLINE_ bool get_slide_on_slope() const { return slide_on_slope; }
	virtual bool allows_one_way_collision() const override { return false; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	//virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // BOX2D_SEPARATION_RAY_SHAPE_2D_H
