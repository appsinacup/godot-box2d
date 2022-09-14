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

void Box2DSpace::body_add_to_state_query_list(SelfList<Box2DBody> *p_body) {
	state_query_list.add(p_body);
}

void Box2DSpace::body_remove_from_state_query_list(SelfList<Box2DBody> *p_body) {
	state_query_list.remove(p_body);
}

void Box2DSpace::call_queries() {
	while (state_query_list.first()) {
		Box2DBody *b = state_query_list.first()->self();
		state_query_list.remove(state_query_list.first());
		b->call_queries();
	}
	// TODO: areas
}

void Box2DSpace::step(float p_step) const {
	const int32 velocityIterations = 10;
	const int32 positionIterations = 8;

	world->Step(p_step, velocityIterations, positionIterations);

	const SelfList<Box2DBody>::List *body_list = &get_active_body_list();
	const SelfList<Box2DBody> *b = body_list->first();
	while (b) {
		b->self()->after_step();
		b = b->next();
	}
}

Box2DSpace::Box2DSpace() {
	b2Vec2 gravity(0.0f, -10.0f);
	world = memnew(b2World(gravity));
}

Box2DSpace::~Box2DSpace() {
	memdelete(world);
}
