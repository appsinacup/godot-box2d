#ifndef BOX2D_COLLISION_OBJECT_H
#define BOX2D_COLLISION_OBJECT_H

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/templates/vector.hpp>

#include <box2d/b2_body.h>

#include "box2d_shape.h"
#include "box2d_space.h"

using namespace godot;

class Box2DCollisionObject {
private:
	RID self;

	b2Body *body = nullptr;
	Box2DSpace* space = nullptr;

	struct Shape {
		Transform2D xform;
		Box2DShape *shape = nullptr;
		b2Fixture *fixture = nullptr;
		bool disabled = false;
	};

	Vector<Shape> shapes;
	Transform2D transform;

	void _update_shapes();

protected:
	void _set_space(Box2DSpace* p_space);

public:
	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	b2Body *get_b2Body() { return body; }
	void set_b2Body(b2Body* p_body) { body = p_body; }

	void add_shape(Box2DShape *p_shape, const Transform2D &p_transform = Transform2D(), bool p_disabled = false);
	void set_shape(int p_index, Box2DShape *p_shape);
	void set_shape_transform(int p_index, const Transform2D &p_transform);

	_FORCE_INLINE_ int get_shape_count() const { return shapes.size(); }
	_FORCE_INLINE_ Box2DShape *get_shape(int p_index) const {
		CRASH_BAD_INDEX(p_index, shapes.size());
		return shapes[p_index].shape;
	}

	_FORCE_INLINE_ const Transform2D &get_shape_transform(int p_index) const {
		CRASH_BAD_INDEX(p_index, shapes.size());
		return shapes[p_index].xform;
	}

	void remove_shape(Box2DShape *p_shape);
	void remove_shape(int p_index);

	virtual void set_space(Box2DSpace* p_space) = 0;
	_FORCE_INLINE_ Box2DSpace *get_space() const { return space; }

	Box2DCollisionObject();
	virtual ~Box2DCollisionObject();
};

#endif // BOX2D_COLLISION_OBJECT_H
