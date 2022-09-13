#include "box2d_space.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>

#include "box2d_collision_object.h"
#include "box2d_body.h"

const SelfList<Box2DBody>::List &Box2DSpace::get_active_body_list() const {
	return active_list;
}

void Box2DSpace::body_add_to_active_list(SelfList<Box2DBody> *p_body) {
	active_list.add(p_body);
}

void Box2DSpace::body_remove_from_active_list(SelfList<Box2DBody> *p_body) {
	active_list.remove(p_body);
}

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
