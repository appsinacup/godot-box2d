#include "physics_server_box2d.h"

#include <godot_cpp/core/class_db.hpp>

#include "box2d_direct_body_state.h"

#define FLUSH_QUERY_CHECK(m_object) \
	ERR_FAIL_COND_MSG(m_object->get_space() && flushing_queries, "Can't change this state while flushing queries. Use call_deferred() or set_deferred() to change monitoring state instead.");

/* SHAPE API */

RID PhysicsServerBox2D::_shape_create(ShapeType p_shape) {
	Box2DShape *shape = nullptr;
	switch (p_shape) {
		case SHAPE_CIRCLE: {
			shape = memnew(Box2DShapeCircle);
		} break;
		case SHAPE_RECTANGLE: {
			shape = memnew(Box2DShapeRectangle);
		} break;
		case SHAPE_CAPSULE: {
			shape = memnew(Box2DShapeCapsule);
		} break;
		case SHAPE_CONVEX_POLYGON: {
			shape = memnew(Box2DShapeConvexPolygon);
		} break;
		case SHAPE_CONCAVE_POLYGON: {
			shape = memnew(Box2DShapeConcavePolygon);
		} break;
	}

	RID id = shape_owner.make_rid(shape);
	shape->set_self(id);

	return id;
}

RID PhysicsServerBox2D::_circle_shape_create() {
	return _shape_create(SHAPE_CIRCLE);
}

RID PhysicsServerBox2D::_rectangle_shape_create() {
	return _shape_create(SHAPE_RECTANGLE);
}

RID PhysicsServerBox2D::_capsule_shape_create() {
	return _shape_create(SHAPE_CAPSULE);
}

RID PhysicsServerBox2D::_convex_polygon_shape_create() {
	return _shape_create(SHAPE_CONVEX_POLYGON);
}

RID PhysicsServerBox2D::_concave_polygon_shape_create() {
	return _shape_create(SHAPE_CONCAVE_POLYGON);
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

/* AREA API */

RID PhysicsServerBox2D::_area_create() {
	Box2DArea *area = memnew(Box2DArea);
	RID rid = area_owner.make_rid(area);
	area->set_self(rid);
	return rid;
}

void PhysicsServerBox2D::_area_set_space(const RID &p_area, const RID &p_space) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	Box2DSpace *space = nullptr;
	if (p_space.is_valid()) {
		space = space_owner.get_or_null(p_space);
		ERR_FAIL_COND(!space);
	}

	if (area->get_space() == space) {
		return; //pointless
	}

	//area->clear_constraints();
	area->set_space(space);
}

RID PhysicsServerBox2D::_area_get_space(const RID &p_area) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, RID());

	Box2DSpace *space = area->get_space();
	if (!space) {
		return RID();
	}
	return space->get_self();
}

void PhysicsServerBox2D::_area_add_shape(const RID &p_area, const RID &p_shape, const Transform2D &p_transform, bool p_disabled) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);

	area->add_shape(shape, p_transform, p_disabled);
}

void PhysicsServerBox2D::_area_set_shape(const RID &p_area, int32_t p_shape_idx, const RID &p_shape) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	ERR_FAIL_COND(!shape->is_configured());

	area->set_shape(p_shape_idx, shape);
}

void PhysicsServerBox2D::_area_set_shape_transform(const RID &p_area, int32_t p_shape_idx, const Transform2D &p_transform) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_shape_transform(p_shape_idx, p_transform);
}

int32_t PhysicsServerBox2D::_area_get_shape_count(const RID &p_area) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, -1);

	return area->get_shape_count();
}

RID PhysicsServerBox2D::_area_get_shape(const RID &p_area, int32_t p_shape_idx) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, RID());

	Box2DShape *shape = area->get_shape(p_shape_idx);
	ERR_FAIL_COND_V(!shape, RID());

	return shape->get_self();
}

Transform2D PhysicsServerBox2D::_area_get_shape_transform(const RID &p_area, int32_t p_shape_idx) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, Transform2D());

	return area->get_shape_transform(p_shape_idx);
}

void PhysicsServerBox2D::_area_remove_shape(const RID &p_area, int32_t p_shape_idx) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->remove_shape(p_shape_idx);
}

void PhysicsServerBox2D::_area_clear_shapes(const RID &p_area) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	while (area->get_shape_count()) {
		area->remove_shape(0);
	}
}

void PhysicsServerBox2D::_area_attach_object_instance_id(const RID &p_area, uint64_t p_id) {
	// TODO: handle default area
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_instance_id(ObjectID(p_id));
}

uint64_t PhysicsServerBox2D::_area_get_object_instance_id(const RID &p_area) const {
	// TODO: handle default area
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, ObjectID());

	return area->get_instance_id();
}

void PhysicsServerBox2D::_area_set_transform(const RID &p_area, const Transform2D &p_transform) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_transform(p_transform);
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

void PhysicsServerBox2D::_body_set_mode(const RID &p_body, BodyMode p_mode) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	FLUSH_QUERY_CHECK(body);

	body->set_mode(p_mode);
};

PhysicsServer2D::BodyMode PhysicsServerBox2D::_body_get_mode(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, BODY_MODE_STATIC);

	return body->get_mode();
};

void PhysicsServerBox2D::_body_add_shape(const RID &p_body, const RID &p_shape, const Transform2D &p_transform, bool p_disabled) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);

	body->add_shape(shape, p_transform, p_disabled);
}

void PhysicsServerBox2D::_body_set_shape(const RID &p_body, int32_t p_shape_idx, const RID &p_shape) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND(!shape);
	ERR_FAIL_COND(!shape->is_configured());

	body->set_shape(p_shape_idx, shape);
}

void PhysicsServerBox2D::_body_set_shape_transform(const RID &p_body, int32_t p_shape_idx, const Transform2D &p_transform) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_transform(p_shape_idx, p_transform);
}

int32_t PhysicsServerBox2D::_body_get_shape_count(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, -1);

	return body->get_shape_count();
}

RID PhysicsServerBox2D::_body_get_shape(const RID &p_body, int32_t p_shape_idx) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, RID());

	Box2DShape *shape = body->get_shape(p_shape_idx);
	ERR_FAIL_COND_V(!shape, RID());

	return shape->get_self();
}

Transform2D PhysicsServerBox2D::_body_get_shape_transform(const RID &p_body, int32_t p_shape_idx) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Transform2D());

	return body->get_shape_transform(p_shape_idx);
}

void PhysicsServerBox2D::_body_remove_shape(const RID &p_body, int32_t p_shape_idx) {
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

void PhysicsServerBox2D::_body_attach_object_instance_id(const RID &p_body, uint64_t p_id) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_instance_id(ObjectID(p_id));
}

uint64_t PhysicsServerBox2D::_body_get_object_instance_id(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, ObjectID());

	return body->get_instance_id();
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

void PhysicsServerBox2D::_body_set_state_sync_callback(const RID &p_body, const Callable &p_callable) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_state_sync_callback(p_callable);
}

PhysicsDirectBodyState2D *PhysicsServerBox2D::_body_get_direct_state(const RID &p_body) {
	ERR_FAIL_COND_V_MSG((using_threads && !doing_sync), nullptr, "Body state is inaccessible right now, wait for iteration or physics process notification.");

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
	else if (area_owner.owns(p_rid)) {
		Box2DArea *area = area_owner.get_or_null(p_rid);
		area_set_space(p_rid, RID());
		area_clear_shapes(p_rid);
		area_owner.free(p_rid);
		memdelete(area);
	}
	else if (body_owner.owns(p_rid)) {
		Box2DBody *body = body_owner.get_or_null(p_rid);
		body_set_space(p_rid, RID());
		body_clear_shapes(p_rid);
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

	for (const Box2DSpace *E : active_spaces) {
		E->step((float)p_step);
	}
}

void PhysicsServerBox2D::_sync() {
	doing_sync = true;
}

void PhysicsServerBox2D::_flush_queries() {
	if (!active) {
		return;
	}

	flushing_queries = true;

	for (const Box2DSpace *E : active_spaces) {
		Box2DSpace *space = const_cast<Box2DSpace *>(E);
		space->call_queries();
	}

	flushing_queries = false;
}

void PhysicsServerBox2D::_end_sync() {
	doing_sync = false;
}

void PhysicsServerBox2D::_finish() {
}

PhysicsServerBox2D::PhysicsServerBox2D() {
}

PhysicsServerBox2D::~PhysicsServerBox2D() {
}