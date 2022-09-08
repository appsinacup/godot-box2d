#include "box2d_space.h"

#include <godot_cpp/core/memory.hpp>

Box2DSpace::Box2DSpace() {
    b2Vec2 gravity(0.0f, -10.0f);
    world = memnew(b2World(gravity));
}

Box2DSpace::~Box2DSpace() {
    memdelete(world);
}
