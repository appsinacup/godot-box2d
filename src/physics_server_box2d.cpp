#include "physics_server_box2d.h"

#include <godot_cpp/core/class_db.hpp>

#include "box2d_direct_body_state.h"

/* SHAPE API */

RID PhysicsServerBox2D::_shape_create(ShapeType p_shape) {
	Box2DShape *shape = nullptr;
	switch (p_shape) {
		case SHAPE_RECTANGLE: {
			shape = memnew(Box2DShapeRectangle);
		}
	}

	RID id = shape_owner.make_rid(shape);
	shape->set_self(id);

	return id;
}

RID PhysicsServerBox2D::_rectangle_shape_create() {
	return _shape_create(SHAPE_RECTANGLE);
}

void PhysicsServerBox2D::_shape_set_data(const RID &p_shape, const Variant &p_data) {
	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	shape->set_data(p_data);
};

Variant PhysicsServerBox2D::_shape_get_data(const RID &p_shape) const {
	const Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND_V(!shape, Variant());
	ERR_FAIL_COND_V(!shape->is_configured(), Variant());
	return shape->get_data();
};

/* SPACE API */

RID PhysicsServerBox2D::_space_create() {
	Box2DSpace *space = memnew(Box2DSpace);
	RID id = space_owner.make_rid(space);
	space->set_self(id);
	// TODO: create area
	return id;
}

void PhysicsServerBox2D::_space_set_active(const RID &p_space, bool p_active) {
	Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND(!space);
	if (p_active) {
		active_spaces.insert(space);
	} else {
		active_spaces.erase(space);
	}
}

bool PhysicsServerBox2D::_space_is_active(const RID &p_space) const {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, false);

	return active_spaces.has(space);
}

/* BODY API */

RID PhysicsServerBox2D::_body_create() {
	Box2DBody *body = memnew(Box2DBody);
	RID rid = body_owner.make_rid(body);
	body->set_self(rid);
	return rid;
}

void PhysicsServerBox2D::_body_set_space(const RID &p_body, const RID &p_space) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	Box2DSpace *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_COND(!space);
	}

	if (body->get_space() == space) {
		return; //pointless
	}

	//body->clear_constraint_list();
	body->set_space(space);
}

RID PhysicsServerBox2D::_body_get_space(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, RID());

	Box2DSpace *space = body->get_space();
	if (!space) {
		return RID();
	}
	return space->get_self();
}

void PhysicsServerBox2D::_body_add_shape(const RID &p_body, const RID &p_shape, const Transform2D &p_transform, bool p_disabled) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);

	body->add_shape(shape, p_transform, p_disabled);
}

void PhysicsServerBox2D::_body_set_shape(const RID &p_body, int64_t p_shape_idx, const RID &p_shape) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	ERR_FAIL_COND(!shape->is_configured());

	body->set_shape(p_shape_idx, shape);
}

void PhysicsServerBox2D::_body_set_shape_transform(const RID &p_body, int64_t p_shape_idx, const Transform2D &p_transform) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_transform(p_shape_idx, p_transform);
}

int64_t PhysicsServerBox2D::_body_get_shape_count(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, -1);

	return body->get_shape_count();
}

RID PhysicsServerBox2D::_body_get_shape(const RID &p_body, int64_t p_shape_idx) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, RID());

	Box2DShape *shape = body->get_shape(p_shape_idx);
	ERR_FAIL_COND_V(!shape, RID());

	return shape->get_self();
}

Transform2D PhysicsServerBox2D::_body_get_shape_transform(const RID &p_body, int64_t p_shape_idx) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Transform2D());

	return body->get_shape_transform(p_shape_idx);
}

void PhysicsServerBox2D::_body_remove_shape(const RID &p_body, int64_t p_shape_idx) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->remove_shape(p_shape_idx);
}

void PhysicsServerBox2D::_body_clear_shapes(const RID &p_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	while (body->get_shape_count()) {
		body->remove_shape(0);
	}
}

void PhysicsServerBox2D::_body_set_state(const RID &p_body, PhysicsServer2D::BodyState p_state, const Variant &p_value) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_state(p_state, p_value);
}

Variant PhysicsServerBox2D::_body_get_state(const RID &p_body, PhysicsServer2D::BodyState p_state) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Variant());

	return body->get_state(p_state);
}

void PhysicsServerBox2D::_body_set_state_sync_callback(const RID &p_body, PhysicsServer2DExtensionStateCallback *p_callback) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_state_sync_callback(p_callback->instance, p_callback->callback);
}

PhysicsDirectBodyState2D *PhysicsServerBox2D::_body_get_direct_state(const RID &p_body) {
	// TODO: check if allowed

	if (!body_owner.owns(p_body)) {
		return nullptr;
	}

	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, nullptr);

	if (!body->get_space()) {
		return nullptr;
	}

	ERR_FAIL_COND_V_MSG(body->get_space()->is_locked(), nullptr, "Body state is inaccessible right now, wait for iteration or physics process notification.");

	return body->get_direct_state();
}

/* MISC */

void PhysicsServerBox2D::_free_rid(const RID &p_rid) {
	if (shape_owner.owns(p_rid)) {
		Box2DShape *shape = shape_owner.get_or_null(p_rid);

		// TODO: remove from owners?

		shape_owner.free(p_rid);
		memdelete(shape);
	}
	else if (body_owner.owns(p_rid)) {
		Box2DBody *body = body_owner.get_or_null(p_rid);

		body_set_space(p_rid, RID());

		// TODO: remove shapes

		body_owner.free(p_rid);
		memdelete(body);
	}
	else if (space_owner.owns(p_rid)) {
		Box2DSpace *space = space_owner.get_or_null(p_rid);
		// TODO: handle objects, area
		active_spaces.erase(space);
		space_owner.free(p_rid);
		memdelete(space);
	}
}

void PhysicsServerBox2D::_set_active(bool p_active) {
	active = p_active;
}

void PhysicsServerBox2D::_init() {
}

void PhysicsServerBox2D::_step(double p_step) {
	if (!active) {
		return;
	}
	// TODO: _update_shapes();

	int32 velocityIterations = 6;
	int32 positionIterations = 2;

	for (const Box2DSpace *E : active_spaces) {
		E->get_b2World()->Step((float)p_step, velocityIterations, positionIterations);
	}
}

void PhysicsServerBox2D::_sync() {
}

void PhysicsServerBox2D::_end_sync() {
}

void PhysicsServerBox2D::_finish() {
}

PhysicsServerBox2D::PhysicsServerBox2D() {
}

PhysicsServerBox2D::~PhysicsServerBox2D() {
}