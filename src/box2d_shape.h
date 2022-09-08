#ifndef BOX2D_SHAPE_H
#define BOX2D_SHAPE_H

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <box2d/b2_polygon_shape.h>

using namespace godot;

class Box2DShape {
	RID self;

public:
	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	Box2DShape() {}
	~Box2DShape() {};
};

class Box2DShapeRectangle: public Box2DShape {
	b2PolygonShape shape;

public:
	Box2DShapeRectangle() {}
	~Box2DShapeRectangle() {};
};

#endif // BOX2D_SHAPE_H
