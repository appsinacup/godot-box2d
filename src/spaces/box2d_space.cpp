#include "box2d_space.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_contact.h>

#include "../bodies/box2d_body.h"
#include "../bodies/box2d_collision_object.h"
#include "box2d_direct_space_state.h"
#include "box2d_space_contact_filter.h"
#include "box2d_space_contact_listener.h"

/* PHYSICS SERVER API */

int Box2DSpace::get_active_body_count() {
	return process_info.active_body_count;
}

int32_t Box2DSpace::get_collision_pairs() {
	return world->GetContactCount();
}
int32_t Box2DSpace::get_island_count() {
	return 0; // not sure if this is exposed
}
int32_t Box2DSpace::get_contact_count() const {
	int32 contact_count = world->GetContactCount();
	int32_t contact_total = 0;
	b2Contact *contacts = world->GetContactList();
	for (int i = 0; i < contact_count; i++) {
		contact_total += contacts->GetManifold()->pointCount;
	}
	return contact_total;
}

PackedVector2Array Box2DSpace::get_contacts() const {
	PackedVector2Array vector_array;
	int32 contact_count = world->GetContactCount();
	int32_t contact_total = 0;
	b2Contact *contacts = world->GetContactList();
	for (int i = 0; i < contact_count; i++) {
		b2WorldManifold worldManifold;
		contacts->GetWorldManifold(&worldManifold);
		for (int j = 0; j < contacts->GetManifold()->pointCount; j++) {
			vector_array.append(box2d_to_godot(worldManifold.points[j]));
		}
	}
	return vector_array;
}

void Box2DSpace::set_debug_contacts(int32_t max_contacts) {
}

void Box2DSpace::set_solver_iterations(int32 p_iterations) {
	solver_iterations = p_iterations;
}

int32 Box2DSpace::get_solver_iterations() const {
	return solver_iterations;
}

void Box2DSpace::step(float p_step) {
	const int32 velocityIterations = solver_iterations;
	const int32 positionIterations = solver_iterations;

	const SelfList<Box2DBody>::List *body_list = &get_active_body_list();
	const SelfList<Box2DBody> *b = body_list->first();
	while (b) {
		b->self()->before_step();
		b = b->next();
	}
	world->Step(p_step, velocityIterations, positionIterations);
	step_count++;

	body_list = &get_active_body_list();
	b = body_list->first();
	process_info.active_body_count = 0;
	while (b) {
		if (!b->self()->is_sleeping()) {
			process_info.active_body_count++;
		}
		b->self()->after_step();
		b = b->next();
	}
}

void Box2DSpace::call_queries() {
	while (state_query_list.first()) {
		Box2DBody *b = state_query_list.first()->self();
		state_query_list.remove(state_query_list.first());
		b->call_queries();
	}
	// TODO: areas
}

Box2DDirectSpaceState *Box2DSpace::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(Box2DDirectSpaceState);
		direct_state->space = this;
	}
	return direct_state;
}
/* BODY API */

const SelfList<Box2DBody>::List &Box2DSpace::get_active_body_list() const {
	return active_list;
}

void Box2DSpace::body_add_to_active_list(SelfList<Box2DBody> *p_body) {
	active_list.add(p_body);
}

void Box2DSpace::body_remove_from_active_list(SelfList<Box2DBody> *p_body) {
	active_list.remove(p_body);
}

void Box2DSpace::body_add_to_state_query_list(SelfList<Box2DBody> *p_body) {
	state_query_list.add(p_body);
}

void Box2DSpace::body_remove_from_state_query_list(SelfList<Box2DBody> *p_body) {
	state_query_list.remove(p_body);
}
/* COLLISION OBJECT API */
void Box2DSpace::add_object(Box2DCollisionObject *p_object) {
	ERR_FAIL_COND(!p_object);
	p_object->set_b2Body(world->CreateBody(p_object->get_b2BodyDef()));
}
void Box2DSpace::remove_object(Box2DCollisionObject *p_object) {
	ERR_FAIL_COND(!p_object);
	world->DestroyBody(p_object->get_b2Body());
	p_object->set_b2Body(nullptr);
	for (Box2DJoint *joint : p_object->get_joints()) {
		joint->set_b2Joint(nullptr); // joint is destroyed when destroying body
	}
}
/* JOINT API */
void Box2DSpace::create_joint(Box2DJoint *joint) {
	remove_joint(joint);
	// create joint once and if both body exist
	if (joint->is_configured()) {
		b2JointDef *joint_def = joint->get_b2JointDef();
		joint->set_b2Joint(world->CreateJoint(joint_def));
	}
}
void Box2DSpace::remove_joint(Box2DJoint *joint) {
	if (joint->get_b2Joint()) {
		world->DestroyJoint(joint->get_b2Joint());
		joint->set_b2Joint(nullptr);
	}
}
/* DIRECT BODY STATE API */

double Box2DSpace::get_step() {
	return world->GetProfile().step;
}

Box2DSpace::Box2DSpace() {
	world = memnew(b2World(b2Vec2_zero)); // gravity comes from areas
	contact_filter = memnew(Box2DSpaceContactFilter);
	contact_listener = memnew(Box2DSpaceContactListener(this));
	world->SetContactFilter(contact_filter);
	world->SetContactListener(contact_listener);
}

Box2DSpace::~Box2DSpace() {
	memdelete(world);
	memdelete(contact_filter);
	memdelete(contact_listener);
	if (direct_state) {
		memdelete(direct_state);
	}
}
