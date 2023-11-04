#ifndef BOX2D_CONCAVE_POLYGON_SHAPE_2D_H
#define BOX2D_CONCAVE_POLYGON_SHAPE_2D_H

#include "box2d_shape_2d.h"

class Box2DConcavePolygonShape2D : public Box2DShape2D {
	struct Segment {
		int points[2] = {};
	};

	LocalVector<Segment> segments;
	LocalVector<Point2> points;

protected:
	virtual b2Shape* create_box2d_shape() const override;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CONCAVE_POLYGON; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override { return 0.0; }
};

#endif // BOX2D_CONCAVE_POLYGON_SHAPE_2D_H
