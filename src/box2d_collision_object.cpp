#include "box2d_collision_object.h"

#include <godot_cpp/core/memory.hpp>

void Box2DCollisionObject::_set_space(Box2DSpace* p_space) {
    // TODO: clean up previous space
    space = p_space;
    // TODO: inform new space
}

Box2DCollisionObject::Box2DCollisionObject() {
}

Box2DCollisionObject::~Box2DCollisionObject() {
}
