#include "physics_server_box2d.h"

#include "../bodies/box2d_direct_body_state.h"
#include "../shapes/box2d_shape_capsule.h"
#include "../shapes/box2d_shape_circle.h"
#include "../shapes/box2d_shape_concave_polygon.h"
#include "../shapes/box2d_shape_convex_polygon.h"
#include "../shapes/box2d_shape_rectangle.h"
#include "../shapes/box2d_shape_segment.h"
#include "../shapes/box2d_shape_separation_ray.h"
#include "../shapes/box2d_shape_world_boundary.h"
#include "../spaces/box2d_direct_space_state.h"

#include <godot_cpp/core/class_db.hpp>

#define FLUSH_QUERY_CHECK(m_object) \
	ERR_FAIL_COND_MSG(m_object->get_space() && flushing_queries, "Can't change this state while flushing queries. Use call_deferred() or set_deferred() to change monitoring state instead.");

using godot::PhysicsServer2D;

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
		case SHAPE_SEGMENT: {
			shape = memnew(Box2DShapeSegment);
		} break;
		case SHAPE_WORLD_BOUNDARY: {
			shape = memnew(Box2DShapeWorldBoundary);
		} break;
		case SHAPE_SEPARATION_RAY: {
			shape = memnew(Box2DShapeSeparationRay);
		} break;
		default: {
			ERR_FAIL_V_MSG(RID(), "UNSUPPORTED");
		} break;
	}

	RID id = shape_owner.make_rid(shape);
	shape->set_self(id);

	return id;
}

RID PhysicsServerBox2D::_world_boundary_shape_create() {
	return _shape_create(SHAPE_WORLD_BOUNDARY);
}

RID PhysicsServerBox2D::_separation_ray_shape_create() {
	return _shape_create(SHAPE_SEPARATION_RAY);
}

RID PhysicsServerBox2D::_segment_shape_create() {
	return _shape_create(SHAPE_SEGMENT);
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
}

void PhysicsServerBox2D::_shape_set_custom_solver_bias(const RID &shape, double bias) {
}

PhysicsServer2D::ShapeType PhysicsServerBox2D::_shape_get_type(const RID &p_shape) const {
	const Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND_V(!shape, SHAPE_CUSTOM);
	return shape->get_type();
}

Variant PhysicsServerBox2D::_shape_get_data(const RID &p_shape) const {
	const Box2DShape *shape = shape_owner.get_or_null(p_shape);
	ERR_FAIL_COND_V(!shape, Variant());
	ERR_FAIL_COND_V(!shape->is_configured(), Variant());
	return shape->get_data();
}

double PhysicsServerBox2D::_shape_get_custom_solver_bias(const RID &shape) const {
	return 0;
}

bool PhysicsServerBox2D::_shape_collide(const RID &p_shape_A, const Transform2D &p_xform_A, const Vector2 &p_motion_A, const RID &p_shape_B, const Transform2D &p_xform_B, const Vector2 &p_motion_B, void *p_results, int32_t p_result_max, int32_t *p_result_count) {
	/*
	const Box2DShape *shape_A = shape_owner.get_or_null(p_shape_A);
	const Box2DShape *shape_B = shape_owner.get_or_null(p_shape_B);
	ERR_FAIL_COND_V(!shape_A, false);
	ERR_FAIL_COND_V(!shape_B, false);
	b2Shape *shapeA = shape_A->get_b2Shape();
	b2Shape *shapeB = shape_B->get_b2Shape();
	ERR_FAIL_COND_V(!shapeA, false);
	ERR_FAIL_COND_V(!shapeB, false);
	b2Transform xfA(godot_to_box2d(p_xform_A.get_origin()), b2Rot(p_xform_A.get_rotation()));
	b2Transform xfB(godot_to_box2d(p_xform_B.get_origin()), b2Rot(p_xform_B.get_rotation()));
	bool overlap = false;
	for (int iA=0;iA<shape_A->get_b2Shape_count();iA++) {
		for (int iB=0;iB<shape_B->get_b2Shape_count();iB++) {
			overlap |= b2TestOverlap(shapeA, iA, shapeB, iB, xfA, xfB);
		}
	}
	*p_result_count = 0;
	return overlap;
	*/
	return false;
}

/* SPACE API */

RID PhysicsServerBox2D::_space_create() {
	Box2DSpace *space = memnew(Box2DSpace);
	RID id = space_owner.make_rid(space);
	space->set_self(id);
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

void PhysicsServerBox2D::_space_set_param(const RID &p_space, PhysicsServer2D::SpaceParameter p_param, double p_value) {
	const Box2DSpace *space_const = space_owner.get_or_null(p_space);
	ERR_FAIL_COND(!space_const);

	switch (p_param) {
		case SPACE_PARAM_SOLVER_ITERATIONS: {
			Box2DSpace *space = const_cast<Box2DSpace *>(space_const);
			space->set_solver_iterations((int32)p_value);
		} break;
		default: {
		}
	}
}

double PhysicsServerBox2D::_space_get_param(const RID &p_space, PhysicsServer2D::SpaceParameter p_param) const {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, 0);

	switch (p_param) {
		case SPACE_PARAM_SOLVER_ITERATIONS:
			return (double)space->get_solver_iterations();
		default: {
		}
	}
	return 0;
}

PhysicsDirectSpaceState2D *PhysicsServerBox2D::_space_get_direct_state(const RID &p_space) {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, nullptr);

	Box2DSpace *space_cast = const_cast<Box2DSpace *>(space);
	return space_cast->get_direct_state();
}

void PhysicsServerBox2D::_space_set_debug_contacts(const RID &p_space, int32_t max_contacts) {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND(!space);
	Box2DSpace *space_cast = const_cast<Box2DSpace *>(space);

	space_cast->set_debug_contacts(max_contacts);
}
PackedVector2Array PhysicsServerBox2D::_space_get_contacts(const RID &p_space) const {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, PackedVector2Array());

	return space->get_contacts();
}
int32_t PhysicsServerBox2D::_space_get_contact_count(const RID &p_space) const {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, 0);

	return space->get_contact_count();
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
		return;
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

void PhysicsServerBox2D::_area_set_shape_disabled(const RID &p_area, int32_t p_shape_idx, bool p_disabled) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_shape_disabled(p_shape_idx, p_disabled);
}

void PhysicsServerBox2D::_area_attach_object_instance_id(const RID &p_area, uint64_t p_id) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	if (!area) {
		area = &default_area;
	}

	area->set_object_instance_id(ObjectID(p_id));
}

uint64_t PhysicsServerBox2D::_area_get_object_instance_id(const RID &p_area) const {
	const Box2DArea *area = area_owner.get_or_null(p_area);
	if (!area) {
		area = &default_area;
	}

	return area->get_object_instance_id();
}

void PhysicsServerBox2D::_area_attach_canvas_instance_id(const RID &p_area, uint64_t p_id) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);

	area->set_canvas_instance_id(ObjectID(p_id));
}

uint64_t PhysicsServerBox2D::_area_get_canvas_instance_id(const RID &p_area) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, ObjectID());

	return area->get_canvas_instance_id();
}

void PhysicsServerBox2D::_area_set_param(const RID &p_area, AreaParameter p_param, const Variant &p_value) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	if (!area) {
		area = &default_area;
	}
	switch (p_param) {
		case AREA_PARAM_GRAVITY_OVERRIDE_MODE: {
			area->set_gravity_override_mode(static_cast<AreaSpaceOverrideMode>((int)p_value));
		} break;
		case AREA_PARAM_GRAVITY: {
			area->set_gravity(variant_to_number(p_value));
		} break;
		case AREA_PARAM_GRAVITY_VECTOR: {
			area->set_gravity_vector(p_value);
		} break;
		case AREA_PARAM_GRAVITY_IS_POINT: {
			area->set_gravity_is_point(p_value);
		} break;
		case AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: {
			area->set_gravity_point_unit_distance(variant_to_number(p_value));
		} break;
		case AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE: {
			area->set_linear_damp_override_mode(static_cast<AreaSpaceOverrideMode>((int)p_value));
		} break;
		case AREA_PARAM_LINEAR_DAMP: {
			area->set_linear_damp(variant_to_number(p_value));
		} break;
		case AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE: {
			area->set_angular_damp_override_mode(static_cast<AreaSpaceOverrideMode>((int)p_value));
		} break;
		case AREA_PARAM_ANGULAR_DAMP: {
			area->set_angular_damp(variant_to_number(p_value));
		} break;
		case AREA_PARAM_PRIORITY: {
			area->set_priority(variant_to_number(p_value));
		} break;
	}
}

void PhysicsServerBox2D::_area_set_transform(const RID &p_area, const Transform2D &p_transform) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_transform(p_transform);
}

Variant PhysicsServerBox2D::_area_get_param(const RID &p_area, PhysicsServer2D::AreaParameter p_param) const {
	const Box2DArea *area = area_owner.get_or_null(p_area);
	if (!area) {
		area = &default_area;
	}
	switch (p_param) {
		case AREA_PARAM_GRAVITY_OVERRIDE_MODE: {
			return area->get_gravity_override_mode();
		} break;
		case AREA_PARAM_GRAVITY: {
			return area->get_gravity();
		} break;
		case AREA_PARAM_GRAVITY_VECTOR: {
			return area->get_gravity_vector();
		} break;
		case AREA_PARAM_GRAVITY_IS_POINT: {
			return area->get_gravity_is_point();
		} break;
		case AREA_PARAM_GRAVITY_POINT_UNIT_DISTANCE: {
			return area->get_gravity_point_unit_distance();
		} break;
		case AREA_PARAM_LINEAR_DAMP_OVERRIDE_MODE: {
			return area->get_linear_damp_override_mode();
		} break;
		case AREA_PARAM_LINEAR_DAMP: {
			return area->get_linear_damp();
		} break;
		case AREA_PARAM_ANGULAR_DAMP_OVERRIDE_MODE: {
			return area->get_angular_damp_override_mode();
		} break;
		case AREA_PARAM_ANGULAR_DAMP: {
			return area->get_angular_damp();
		} break;
		case AREA_PARAM_PRIORITY: {
			return area->get_priority();
		} break;
	}
	return Variant();
}

Transform2D PhysicsServerBox2D::_area_get_transform(const RID &p_area) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, Transform2D());
	return area->get_transform();
}

void PhysicsServerBox2D::_area_set_collision_layer(const RID &p_area, uint32_t p_layer) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_collision_layer(p_layer);
}
uint32_t PhysicsServerBox2D::_area_get_collision_layer(const RID &p_area) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, 0);
	return area->get_collision_layer();
}
void PhysicsServerBox2D::_area_set_collision_mask(const RID &p_area, uint32_t p_mask) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_collision_mask(p_mask);
}
uint32_t PhysicsServerBox2D::_area_get_collision_mask(const RID &p_area) const {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, 0);
	return area->get_collision_mask();
}
void PhysicsServerBox2D::_area_set_monitorable(const RID &p_area, bool p_monitorable) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_monitorable(p_monitorable);
}
void PhysicsServerBox2D::_area_set_pickable(const RID &p_area, bool p_pickable) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_pickable(p_pickable);
}
void PhysicsServerBox2D::_area_set_monitor_callback(const RID &p_area, const Callable &p_callback) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_monitor_callback(p_callback);
}
void PhysicsServerBox2D::_area_set_area_monitor_callback(const RID &p_area, const Callable &p_callback) {
	Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND(!area);
	area->set_area_monitor_callback(p_callback);
}

/* BODY API */

RID PhysicsServerBox2D::_body_create() {
	Box2DBody *body = memnew(Box2DBody);
	RID rid = body_owner.make_rid(body);
	body->set_self(rid);
	default_area.add_body(body);
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

void PhysicsServerBox2D::_body_set_shape_disabled(const RID &p_body, int32_t p_shape_idx, bool p_disabled) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_disabled(p_shape_idx, p_disabled);
}

void PhysicsServerBox2D::_body_set_shape_as_one_way_collision(const RID &p_body, int32_t shape_idx, bool enable, double margin) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_shape_as_one_way_collision(shape_idx, enable);
}

void PhysicsServerBox2D::_body_attach_object_instance_id(const RID &p_body, uint64_t p_id) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_object_instance_id(ObjectID(p_id));
}

uint64_t PhysicsServerBox2D::_body_get_object_instance_id(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, ObjectID());

	return body->get_object_instance_id();
}

void PhysicsServerBox2D::_body_attach_canvas_instance_id(const RID &p_body, uint64_t p_id) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	body->set_canvas_instance_id(ObjectID(p_id));
}
uint64_t PhysicsServerBox2D::_body_get_canvas_instance_id(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, ObjectID());

	return body->get_canvas_instance_id();
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

void PhysicsServerBox2D::_body_set_continuous_collision_detection_mode(const RID &p_body, PhysicsServer2D::CCDMode p_mode) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_continuous_collision_detection_mode(p_mode);
}
PhysicsServer2D::CCDMode PhysicsServerBox2D::_body_get_continuous_collision_detection_mode(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, PhysicsServer2D::CCDMode::CCD_MODE_DISABLED);
	return body->get_continuous_collision_detection_mode();
}
void PhysicsServerBox2D::_body_set_collision_layer(const RID &p_body, uint32_t p_layer) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_collision_layer(p_layer);
}

uint32_t PhysicsServerBox2D::_body_get_collision_layer(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->get_collision_layer();
}
void PhysicsServerBox2D::_body_set_collision_mask(const RID &p_body, uint32_t p_mask) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_collision_mask(p_mask);
}
uint32_t PhysicsServerBox2D::_body_get_collision_mask(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->get_collision_mask();
}
void PhysicsServerBox2D::_body_set_collision_priority(const RID &p_body, double p_priority) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_priority(p_priority);
}
double PhysicsServerBox2D::_body_get_collision_priority(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->get_priority();
}
void PhysicsServerBox2D::_body_set_param(const RID &p_body, PhysicsServer2D::BodyParameter p_param, const Variant &p_value) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	switch (p_param) {
		case BODY_PARAM_BOUNCE: {
			body->set_bounce(variant_to_number(p_value));
		}
		case BODY_PARAM_FRICTION: {
			body->set_friction(variant_to_number(p_value));
		}
		case BODY_PARAM_MASS: {
			body->set_mass(variant_to_number(p_value));
		} break;
		case BODY_PARAM_INERTIA: {
			body->set_inertia(variant_to_number(p_value));
		} break;
		case BODY_PARAM_CENTER_OF_MASS: {
			body->set_center_of_mass(p_value);
		} break;
		case BODY_PARAM_GRAVITY_SCALE: {
			body->set_gravity_scale(variant_to_number(p_value));
		} break;
		case BODY_PARAM_LINEAR_DAMP: {
			body->set_linear_damp(variant_to_number(p_value));
		} break;
		case BODY_PARAM_LINEAR_DAMP_MODE: {
			body->set_linear_damp_mode(static_cast<BodyDampMode>((int)p_value));
		} break;
		case BODY_PARAM_ANGULAR_DAMP: {
			body->set_angular_damp(variant_to_number(p_value));
		} break;
		case BODY_PARAM_ANGULAR_DAMP_MODE: {
			body->set_angular_damp_mode(static_cast<BodyDampMode>((int)p_value));
		} break;
		default: {
		}
	}
}
Variant PhysicsServerBox2D::_body_get_param(const RID &p_body, PhysicsServer2D::BodyParameter p_param) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Variant());
	switch (p_param) {
		case BODY_PARAM_BOUNCE: {
			return body->get_bounce();
		}
		case BODY_PARAM_FRICTION: {
			return body->get_friction();
		}
		case BODY_PARAM_MASS: {
			return body->get_mass();
		}
		case BODY_PARAM_INERTIA: {
			return body->get_inertia();
		}
		case BODY_PARAM_CENTER_OF_MASS: {
			return body->get_center_of_mass();
		}
		case BODY_PARAM_GRAVITY_SCALE: {
			return body->get_gravity_scale();
		}
		case BODY_PARAM_LINEAR_DAMP_MODE: {
			return body->get_linear_damp_mode();
		}
		case BODY_PARAM_LINEAR_DAMP: {
			return body->get_linear_damp();
		}
		case BODY_PARAM_ANGULAR_DAMP_MODE: {
			return body->get_angular_damp_mode();
		}
		case BODY_PARAM_ANGULAR_DAMP: {
			return body->get_angular_damp();
		}
		default: {
		}
	}
	return Variant();
}
void PhysicsServerBox2D::_body_reset_mass_properties(const RID &p_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->reset_mass_properties();
}
void PhysicsServerBox2D::_body_apply_central_impulse(const RID &p_body, const Vector2 &p_impulse) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->apply_central_impulse(p_impulse);
}
void PhysicsServerBox2D::_body_apply_torque_impulse(const RID &p_body, double p_impulse) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->apply_torque_impulse(p_impulse);
}
void PhysicsServerBox2D::_body_apply_impulse(const RID &p_body, const Vector2 &p_impulse, const Vector2 &p_position) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->apply_impulse(p_impulse, p_position);
}
void PhysicsServerBox2D::_body_apply_central_force(const RID &p_body, const Vector2 &p_force) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->apply_central_force(p_force);
}
void PhysicsServerBox2D::_body_apply_force(const RID &p_body, const Vector2 &p_force, const Vector2 &p_position) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->apply_force(p_force, p_position);
}
void PhysicsServerBox2D::_body_apply_torque(const RID &p_body, double p_torque) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->apply_torque(p_torque);
}
void PhysicsServerBox2D::_body_add_constant_central_force(const RID &p_body, const Vector2 &p_force) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->add_constant_central_force(p_force);
}
void PhysicsServerBox2D::_body_add_constant_force(const RID &p_body, const Vector2 &p_force, const Vector2 &p_position) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->add_constant_force(p_force, p_position);
}
void PhysicsServerBox2D::_body_add_constant_torque(const RID &p_body, double p_torque) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->add_constant_torque(p_torque);
}
void PhysicsServerBox2D::_body_set_constant_force(const RID &p_body, const Vector2 &p_force) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_constant_force(p_force);
}
Vector2 PhysicsServerBox2D::_body_get_constant_force(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, Vector2());
	return body->get_constant_force();
}
void PhysicsServerBox2D::_body_set_constant_torque(const RID &p_body, double p_torque) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_constant_torque(p_torque);
}
double PhysicsServerBox2D::_body_get_constant_torque(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->get_constant_torque();
}
void PhysicsServerBox2D::_body_set_axis_velocity(const RID &p_body, const Vector2 &p_axis_velocity) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_linear_velocity(p_axis_velocity);
}
void PhysicsServerBox2D::_body_add_collision_exception(const RID &p_body, const RID &p_excepted_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	Box2DBody *excepted_body = body_owner.get_or_null(p_excepted_body);
	ERR_FAIL_COND(!body);
	ERR_FAIL_COND(!excepted_body);
	body->add_collision_exception(body);
}
void PhysicsServerBox2D::_body_remove_collision_exception(const RID &p_body, const RID &p_excepted_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	Box2DBody *excepted_body = body_owner.get_or_null(p_excepted_body);
	ERR_FAIL_COND(!body);
	ERR_FAIL_COND(!excepted_body);
	body->remove_collision_exception(body);
}
TypedArray<RID> PhysicsServerBox2D::_body_get_collision_exceptions(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, TypedArray<RID>());
	return body->get_collision_exception();
}

void PhysicsServerBox2D::_body_set_max_contacts_reported(const RID &p_body, int32_t p_amount) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_max_contacts_reported(p_amount);
}
int32_t PhysicsServerBox2D::_body_get_max_contacts_reported(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, 0);
	return body->get_max_contacts_reported();
}
void PhysicsServerBox2D::_body_set_contacts_reported_depth_threshold(const RID &p_body, double p_threshold) {
}
double PhysicsServerBox2D::_body_get_contacts_reported_depth_threshold(const RID &p_body) const {
	return 0;
}
void PhysicsServerBox2D::_body_set_omit_force_integration(const RID &p_body, bool p_enable) {
}
bool PhysicsServerBox2D::_body_is_omitting_force_integration(const RID &p_body) const {
	return false;
}
void PhysicsServerBox2D::_body_set_force_integration_callback(const RID &p_body, const Callable &p_callable, const Variant &p_userdata) {
}
bool PhysicsServerBox2D::_body_collide_shape(const RID &p_body, int32_t p_body_shape, const RID &p_shape, const Transform2D &p_shape_xform, const Vector2 &p_motion, void *p_results, int32_t p_result_max, int32_t *p_result_count) {
	WARN_PRINT_ONCE("TODO_body_collide_shape");
	return false;
}
void PhysicsServerBox2D::_body_set_pickable(const RID &p_body, bool p_pickable) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	return body->set_pickable(p_pickable);
}
bool PhysicsServerBox2D::_body_test_motion(const RID &p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *p_result) const {
	WARN_PRINT_ONCE("TODO_body_test_motion");
	return false;
}

/* JOINT API */

RID PhysicsServerBox2D::_joint_create() {
	Box2DJoint *joint = memnew(Box2DJoint);
	ERR_FAIL_COND_V(!joint, RID());
	RID id = joint_owner.make_rid(joint);
	joint->set_self(id);
	return id;
}
void PhysicsServerBox2D::_joint_clear(const RID &p_joint) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);
	joint->clear();
}
void PhysicsServerBox2D::_joint_set_param(const RID &p_joint, PhysicsServer2D::JointParam p_param, double p_value) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);
	switch (p_param) {
		case JOINT_PARAM_BIAS:
			joint->set_bias(p_value);
			break;
		case JOINT_PARAM_MAX_BIAS:
			joint->set_max_bias(p_value);
			break;
		case JOINT_PARAM_MAX_FORCE:
			joint->set_max_force(p_value);
			break;
	}
}
double PhysicsServerBox2D::_joint_get_param(const RID &p_joint, PhysicsServer2D::JointParam p_param) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, 0);
	switch (p_param) {
		case JOINT_PARAM_BIAS:
			return joint->get_bias();
		case JOINT_PARAM_MAX_BIAS:
			return joint->get_max_bias();
		case JOINT_PARAM_MAX_FORCE:
			return joint->get_max_force();
	}
	return 0;
}
void PhysicsServerBox2D::_joint_disable_collisions_between_bodies(const RID &p_joint, bool p_disable) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);
	return joint->set_disable_collisions(p_disable);
}
bool PhysicsServerBox2D::_joint_is_disabled_collisions_between_bodies(const RID &p_joint) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, false);
	return joint->get_disable_collisions();
}
void PhysicsServerBox2D::_joint_make_pin(const RID &p_joint, const Vector2 &p_anchor, const RID &p_body_a, const RID &p_body_b) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	Box2DBody *body_a = body_owner.get_or_null(p_body_a);
	Box2DBody *body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(!body_a);
	ERR_FAIL_COND(!body_b);
	ERR_FAIL_COND(!joint);
	joint->make_pin(p_anchor, body_a, body_b);
	body_a->add_joint(joint);
	body_b->add_joint(joint);
	Box2DSpace *space = body_a->get_space();
	ERR_FAIL_COND(!space);
	joint->set_space(space);
	space->create_joint(joint);
}

void PhysicsServerBox2D::_joint_make_groove(const RID &p_joint, const Vector2 &p_a_groove1, const Vector2 &p_a_groove2, const Vector2 &p_b_anchor, const RID &p_body_a, const RID &p_body_b) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	Box2DBody *body_a = body_owner.get_or_null(p_body_a);
	Box2DBody *body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(!body_a);
	ERR_FAIL_COND(!body_b);
	ERR_FAIL_COND(!joint);
	joint->make_groove(p_a_groove1, p_a_groove2, p_b_anchor, body_a, body_b);
	body_a->add_joint(joint);
	body_b->add_joint(joint);
	Box2DSpace *space = body_a->get_space();
	ERR_FAIL_COND(!space);
	joint->set_space(space);
	space->create_joint(joint);
}
void PhysicsServerBox2D::_joint_make_damped_spring(const RID &p_joint, const Vector2 &p_anchor_a, const Vector2 &p_anchor_b, const RID &p_body_a, const RID &p_body_b) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	Box2DBody *body_a = body_owner.get_or_null(p_body_a);
	Box2DBody *body_b = body_owner.get_or_null(p_body_b);
	ERR_FAIL_COND(!body_a);
	ERR_FAIL_COND(!body_b);
	ERR_FAIL_COND(!joint);
	joint->make_damped_spring(p_anchor_a, p_anchor_b, body_a, body_b);
	body_a->add_joint(joint);
	body_b->add_joint(joint);
	Box2DSpace *space = body_a->get_space();
	ERR_FAIL_COND(!space);
	joint->set_space(space);
	space->create_joint(joint);
}
void PhysicsServerBox2D::_pin_joint_set_param(const RID &p_joint, PhysicsServer2D::PinJointParam p_param, double p_value) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);
	switch (p_param) {
		case PIN_JOINT_SOFTNESS: {
			joint->set_pin_softness(p_value);
		} break;
	}
}
double PhysicsServerBox2D::_pin_joint_get_param(const RID &p_joint, PhysicsServer2D::PinJointParam p_param) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, 0);
	switch (p_param) {
		case PIN_JOINT_SOFTNESS: {
			return joint->get_pin_softness();
		} break;
	}
	return 0;
}
void PhysicsServerBox2D::_damped_spring_joint_set_param(const RID &p_joint, PhysicsServer2D::DampedSpringParam p_param, double p_value) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);
	switch (p_param) {
		case DAMPED_SPRING_REST_LENGTH: {
			joint->set_damped_spring_rest_length(p_value);
		} break;
		case DAMPED_SPRING_STIFFNESS: {
			joint->set_damped_spring_stiffness(p_value);
		} break;
		case DAMPED_SPRING_DAMPING: {
			joint->set_damped_spring_damping(p_value);
		} break;
	}
}
double PhysicsServerBox2D::_damped_spring_joint_get_param(const RID &p_joint, PhysicsServer2D::DampedSpringParam p_param) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, 0);
	switch (p_param) {
		case DAMPED_SPRING_REST_LENGTH: {
			return joint->get_damped_spring_rest_length();
		} break;
		case DAMPED_SPRING_STIFFNESS: {
			return joint->get_damped_spring_stiffness();
		} break;
		case DAMPED_SPRING_DAMPING: {
			return joint->get_damped_spring_damping();
		} break;
	}
	return 0;
}
PhysicsServer2D::JointType PhysicsServerBox2D::_joint_get_type(const RID &p_joint) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, PhysicsServer2D::JointType::JOINT_TYPE_PIN);
	return joint->get_type();
}

/* MISC */

void PhysicsServerBox2D::_free_rid(const RID &p_rid) {
	if (shape_owner.owns(p_rid)) {
		Box2DShape *shape = shape_owner.get_or_null(p_rid);

		// TODO: remove from owners?

		shape_owner.free(p_rid);
		memdelete(shape);
	} else if (area_owner.owns(p_rid)) {
		Box2DArea *area = area_owner.get_or_null(p_rid);
		area_set_space(p_rid, RID());
		area_clear_shapes(p_rid);
		area_owner.free(p_rid);
		memdelete(area);
	} else if (body_owner.owns(p_rid)) {
		Box2DBody *body = body_owner.get_or_null(p_rid);
		body_set_space(p_rid, RID());
		body_clear_shapes(p_rid);
		body_owner.free(p_rid);
		memdelete(body);
	} else if (space_owner.owns(p_rid)) {
		Box2DSpace *space = space_owner.get_or_null(p_rid);
		// TODO: handle objects, area
		active_spaces.erase(space);
		space_owner.free(p_rid);
		memdelete(space);
	} else if (joint_owner.owns(p_rid)) {
		Box2DJoint *joint = joint_owner.get_or_null(p_rid);
		joint_owner.free(p_rid);
		memdelete(joint);
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
		Box2DSpace *space = const_cast<Box2DSpace *>(E);
		space->step((float)p_step);
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
bool PhysicsServerBox2D::_is_flushing_queries() const {
	return flushing_queries;
}

int PhysicsServerBox2D::_get_process_info(ProcessInfo process_info) {
	switch (process_info) {
		case INFO_ACTIVE_OBJECTS: {
			int active_body_count = 0;
			for (const Box2DSpace *E : active_spaces) {
				Box2DSpace *space = const_cast<Box2DSpace *>(E);
				active_body_count += space->get_active_body_count();
			}
			return active_body_count;
		} break;
		case INFO_COLLISION_PAIRS: {
			int collision_pairs = 0;
			for (const Box2DSpace *E : active_spaces) {
				Box2DSpace *space = const_cast<Box2DSpace *>(E);
				collision_pairs += space->get_collision_pairs();
			}
			return collision_pairs;
		} break;
		case INFO_ISLAND_COUNT: {
			int islands = 0;
			for (const Box2DSpace *E : active_spaces) {
				Box2DSpace *space = const_cast<Box2DSpace *>(E);
				islands += space->get_island_count();
			}
			return islands;
		} break;
	}
	return 0;
}

PhysicsServerBox2D::PhysicsServerBox2D() {
	default_area.set_priority(-1);
	default_area.set_gravity_override_mode(AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE);
	default_area.set_linear_damp_override_mode(AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE);
	default_area.set_angular_damp_override_mode(AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE);
}

PhysicsServerBox2D::~PhysicsServerBox2D() {
}
