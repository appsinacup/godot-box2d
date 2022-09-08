#ifndef BOX2D_COLLISION_OBJECT_H
#define BOX2D_COLLISION_OBJECT_H

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <box2d/b2_body.h>

#include "box2d_space.h"

using namespace godot;

class Box2DCollisionObject {
private:
	RID self;

	b2Body *body = nullptr;
	Box2DSpace* space = nullptr;

protected:
	void _set_space(Box2DSpace* p_space);

public:
	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	b2Body *get_b2Body() { return body; }
	void set_b2Body(b2Body* p_body) { body = p_body; }

	virtual void set_space(Box2DSpace* p_space) = 0;
	_FORCE_INLINE_ Box2DSpace *get_space() const { return space; }

	Box2DCollisionObject();
	virtual ~Box2DCollisionObject();
};

#endif // BOX2D_COLLISION_OBJECT_H
