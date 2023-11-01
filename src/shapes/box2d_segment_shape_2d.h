#ifndef BOX2D_SEGMENT_SHAPE_2D_H
#define BOX2D_SEGMENT_SHAPE_2D_H

#include "box2d_shape_2d.h"

class Box2DSegmentShape2D : public Box2DShape2D {
	Vector2 a;
	Vector2 b;
	Vector2 n;

protected:
	virtual box2d::Handle create_box2d_shape() const override;

public:
	_FORCE_INLINE_ const Vector2 &get_a() const { return a; }
	_FORCE_INLINE_ const Vector2 &get_b() const { return b; }
	_FORCE_INLINE_ const Vector2 &get_normal() const { return n; }

	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_SEGMENT; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;
};

#endif // BOX2D_SEGMENT_SHAPE_2D_H
