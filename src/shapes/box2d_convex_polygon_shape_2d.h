#ifndef BOX2D_CONVEX_POLYGON_SHAPE_2D_H
#define BOX2D_CONVEX_POLYGON_SHAPE_2D_H

#include "box2d_shape_2d.h"

class Box2DConvexPolygonShape2D : public Box2DShape2D {
	struct Point {
		Vector2 pos;
		Vector2 normal; //normal to next segment
	};

	Point *points = nullptr;
	int point_count = 0;

public:
	virtual PhysicsServer2D::ShapeType get_type() const override { return PhysicsServer2D::SHAPE_CONVEX_POLYGON; }

	virtual void set_data(const Variant &p_data) override;
	virtual Variant get_data() const override;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const override;

	~Box2DConvexPolygonShape2D();

protected:
	virtual box2d::ShapeHandle create_box2d_shape() const override;
};

#endif // BOX2D_CONVEX_POLYGON_SHAPE_2D_H
