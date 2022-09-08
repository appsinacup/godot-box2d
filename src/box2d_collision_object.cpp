#include "box2d_collision_object.h"

#include <godot_cpp/core/memory.hpp>

void Box2DCollisionObject::_set_space(Box2DSpace* p_space) {
	if (space) {
		space->remove_object(this);
		// TODO: remove shapes?
	}
	space = p_space;
	if (space) {
		space->add_object(this);
		// TODO: update shapes?
	}
}

Box2DCollisionObject::Box2DCollisionObject() {
}

Box2DCollisionObject::~Box2DCollisionObject() {
}
