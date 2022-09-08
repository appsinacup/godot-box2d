#include "box2d_space.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>

#include "box2d_collision_object.h"

void Box2DSpace::add_object(Box2DCollisionObject *p_object) {
	ERR_FAIL_COND(!p_object);
	b2BodyDef body_def;
	p_object->set_b2Body(world->CreateBody(&body_def));
}

void Box2DSpace::remove_object(Box2DCollisionObject *p_object) {
	ERR_FAIL_COND(!p_object);
	world->DestroyBody(p_object->get_b2Body());
}

Box2DSpace::Box2DSpace() {
	b2Vec2 gravity(0.0f, -10.0f);
	world = memnew(b2World(gravity));
}

Box2DSpace::~Box2DSpace() {
	memdelete(world);
}
