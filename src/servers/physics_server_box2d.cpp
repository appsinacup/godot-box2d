#include "physics_server_box2d.h"

#include "../b2_user_settings.h"

#include "../spaces/box2d_space_contact_listener.h"
#include "../spaces/box2d_sweep_test.h"

#include "../bodies/box2d_direct_body_state.h"
#include "../shapes/box2d_shape.h"
#include "../shapes/box2d_shape_capsule.h"
#include "../shapes/box2d_shape_circle.h"
#include "../shapes/box2d_shape_concave_polygon.h"
#include "../shapes/box2d_shape_convex_polygon.h"
#include "../shapes/box2d_shape_rectangle.h"
#include "../shapes/box2d_shape_segment.h"
#include "../shapes/box2d_shape_separation_ray.h"
#include "../shapes/box2d_shape_world_boundary.h"
#include "../spaces/box2d_direct_space_state.h"

#include <godot_cpp/classes/physics_shape_query_parameters2d.hpp>
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
			ERR_FAIL_V_MSG(RID(), "UNSUPPORTED shape.");
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
	WARN_PRINT_ONCE("_shape_set_custom_solver_bias is UNUSED");
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
	WARN_PRINT_ONCE("_shape_get_custom_solver_bias is UNUSED");
	return 0;
}

bool PhysicsServerBox2D::_shape_collide(const RID &p_shape_A, const Transform2D &p_xform_A, const Vector2 &p_motion_A, const RID &p_shape_B, const Transform2D &p_xform_B, const Vector2 &p_motion_B, void *p_results, int32_t p_result_max, int32_t *p_result_count) {
	Box2DShape *shape_A = shape_owner.get_or_null(p_shape_A);
	Box2DShape *shape_B = shape_owner.get_or_null(p_shape_B);
	ERR_FAIL_COND_V(!shape_A, false);
	ERR_FAIL_COND_V(!shape_B, false);
	b2Transform shape_A_transform(godot_to_box2d(p_xform_A.get_origin()), b2Rot(p_xform_A.get_rotation()));
	b2Transform shape_B_transform(godot_to_box2d(p_xform_B.get_origin()), b2Rot(p_xform_B.get_rotation()));
	b2Sweep sweepA = Box2DSweepTest::create_b2_sweep(shape_A_transform, b2Vec2_zero, godot_to_box2d(p_motion_A));
	b2Sweep sweepB = Box2DSweepTest::create_b2_sweep(shape_B_transform, b2Vec2_zero, godot_to_box2d(p_motion_B));
	Vector<SweepTestResult> sweep_results;
	Transform2D identity;
	for (int i = 0; i < shape_A->get_b2Shape_count(false); i++) {
		Box2DShape::ShapeInfo shape_info_A{ i, identity, false, false };
		b2Shape *b2_shape_A = shape_A->get_transformed_b2Shape(shape_info_A, nullptr);

		for (int j = 0; j < shape_A->get_b2Shape_count(false); j++) {
			Box2DShape::ShapeInfo shape_info_B{ i, identity, false, false };
			b2Shape *b2_shape_B = shape_B->get_transformed_b2Shape(shape_info_B, nullptr);
			SweepShape sweep_shape_A{ shape_A, sweepA, nullptr, shape_A_transform };
			SweepShape sweep_shape_B{ shape_B, sweepB, nullptr, shape_B_transform };
			Vector<SweepTestResult> output = Box2DSweepTest::shape_cast(sweep_shape_A, b2_shape_A, sweep_shape_B, b2_shape_B, 0);
			if (!output.is_empty()) {
				sweep_results.append_array(output);
			}
			shape_B->erase_shape(b2_shape_B);
			memdelete(b2_shape_B);
		}
		shape_A->erase_shape(b2_shape_A);
		memdelete(b2_shape_A);
	}
	auto *results = static_cast<Vector2 *>(p_results);
	int count = 0;
	for (SweepTestResult sweep_result : sweep_results) {
		for (int i = 0; i < sweep_result.manifold.pointCount; i++) {
			if (count >= p_result_max) {
				return !sweep_results.is_empty();
			}
			results[count++] = box2d_to_godot(sweep_result.world_manifold.points[i]);
		}
	}
	*p_result_count = count;
	return !sweep_results.is_empty();
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
	// UNUSED
}

double PhysicsServerBox2D::_space_get_param(const RID &p_space, PhysicsServer2D::SpaceParameter p_param) const {
	// UNUSED
	return 0;
}

PhysicsDirectSpaceState2D *PhysicsServerBox2D::_space_get_direct_state(const RID &p_space) {
	const Box2DSpace *space = space_owner.get_or_null(p_space);
	ERR_FAIL_COND_V(!space, nullptr);

	Box2DSpace *space_cast = const_cast<Box2DSpace *>(space);
	return space_cast->get_direct_state(this);
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
	ERR_FAIL_COND(!area);

	area->set_object_instance_id(ObjectID(p_id));
}

uint64_t PhysicsServerBox2D::_area_get_object_instance_id(const RID &p_area) const {
	const Box2DArea *area = area_owner.get_or_null(p_area);
	ERR_FAIL_COND_V(!area, ObjectID());

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
	if (space_owner.owns(p_area)) {
		area = space_owner.get_or_null(p_area)->get_default_area();
	}
	ERR_FAIL_COND(!area);
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
	if (space_owner.owns(p_area)) {
		area = space_owner.get_or_null(p_area)->get_default_area();
	}
	ERR_FAIL_COND_V(!area, Variant());
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
		return;
	}
	if (space) {
		space->get_default_area()->add_body(body);
	}
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
	Box2DShape *shape = body->get_shape(p_shape_idx);
	ERR_FAIL_COND(!shape);
	body->remove_shape(p_shape_idx);
}

void PhysicsServerBox2D::_body_clear_shapes(const RID &p_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);

	while (body->get_shape_count()) {
		Box2DShape *shape = body->get_shape(0);
		ERR_FAIL_COND(!shape);
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
		} break;
		case BODY_PARAM_FRICTION: {
			body->set_friction(variant_to_number(p_value));
		} break;
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

	Vector2 v = body->get_direct_state()->get_linear_velocity();
	Vector2 axis = p_axis_velocity.normalized();
	v -= axis * axis.dot(v);
	v += p_axis_velocity;
	body->get_direct_state()->set_linear_velocity(v);
}
void PhysicsServerBox2D::_body_add_collision_exception(const RID &p_body, const RID &p_excepted_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	Box2DBody *excepted_body = body_owner.get_or_null(p_excepted_body);
	ERR_FAIL_COND(!body);
	ERR_FAIL_COND(!excepted_body);
	body->add_collision_exception(excepted_body);
}
void PhysicsServerBox2D::_body_remove_collision_exception(const RID &p_body, const RID &p_excepted_body) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	Box2DBody *excepted_body = body_owner.get_or_null(p_excepted_body);
	ERR_FAIL_COND(!body);
	ERR_FAIL_COND(!excepted_body);
	body->remove_collision_exception(excepted_body);
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
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_omit_force_integration(p_enable);
}
bool PhysicsServerBox2D::_body_is_omitting_force_integration(const RID &p_body) const {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, false);
	return body->is_omitting_force_integration();
}
void PhysicsServerBox2D::_body_set_force_integration_callback(const RID &p_body, const Callable &p_callable, const Variant &p_userdata) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	body->set_force_integration_callback(p_callable, p_userdata);
}
bool PhysicsServerBox2D::_body_collide_shape(const RID &p_body, int32_t p_body_shape, const RID &p_shape, const Transform2D &p_shape_xform, const Vector2 &p_motion, void *p_results, int32_t p_result_max, int32_t *p_result_count) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND_V(!body, false);
	Box2DShape *shape = shape_owner.get_or_null(p_shape);
	if (!shape) {
		shape = body->get_shape(p_body_shape);
		ERR_FAIL_COND_V(!shape, false);
	}
	Box2DDirectSpaceState *space_state = (Box2DDirectSpaceState *)body->get_space_state();
	ERR_FAIL_COND_V(!space_state, false);
	return space_state->_collide_shape(shape->get_self(), p_shape_xform, p_motion, 0, body->get_collision_mask(), true, false, p_results, p_result_max, p_result_count);
}
void PhysicsServerBox2D::_body_set_pickable(const RID &p_body, bool p_pickable) {
	Box2DBody *body = body_owner.get_or_null(p_body);
	ERR_FAIL_COND(!body);
	return body->set_pickable(p_pickable);
}
Vector<SweepTestResult> remove_disabled_and_one_way_wrong_direction(Vector<SweepTestResult> sweep_test_results, Box2DBody *body, const Vector2 &p_motion) {
	for (int i = 0; i < sweep_test_results.size(); i++) {
		Box2DCollisionObject *body_B = sweep_test_results[i].sweep_shape_B.fixture->GetBody()->GetUserData().collision_object;
		// collision exception
		if (!body_B || body->is_body_collision_excepted(body_B) || body_B->is_body_collision_excepted(body)) {
			sweep_test_results.remove_at(i);
			i--;
			continue;
		}
		if (body_B != nullptr) {
			ERR_CONTINUE(!sweep_test_results[i].sweep_shape_A.fixture);
			ERR_CONTINUE(!sweep_test_results[i].sweep_shape_B.fixture);
			b2FixtureUserData fixtureA_user_data = sweep_test_results[i].sweep_shape_A.fixture->GetUserData();
			b2FixtureUserData fixtureB_user_data = sweep_test_results[i].sweep_shape_B.fixture->GetUserData();
			b2Vec2 one_way_collision_direction_A(fixtureA_user_data.one_way_collision_direction_x, fixtureA_user_data.one_way_collision_direction_y);
			b2Vec2 one_way_collision_direction_B(fixtureB_user_data.one_way_collision_direction_x, fixtureB_user_data.one_way_collision_direction_y);
			bool one_way_collision_A = fixtureA_user_data.one_way_collision;
			bool one_way_collision_B = fixtureB_user_data.one_way_collision;
			bool should_disable_collision = false;
			if (one_way_collision_A) {
				should_disable_collision = Box2DSpaceContactListener::should_disable_collision_one_way_direction(one_way_collision_direction_A, body->get_b2Body(), body_B->get_b2Body(), body_B->get_b2Body()->GetLinearVelocity());
			}
			if (one_way_collision_B && !should_disable_collision) {
				should_disable_collision = Box2DSpaceContactListener::should_disable_collision_one_way_direction(one_way_collision_direction_B, body_B->get_b2Body(), body->get_b2Body(), godot_to_box2d(p_motion));
			}
			if (should_disable_collision) {
				sweep_test_results.remove_at(i);
				i--;
			}
		}
	}
	return sweep_test_results;
}

#define TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR 0.05
#define BODY_MOTION_RECOVER_ATTEMPTS 4
#define BODY_MOTION_RECOVER_RATIO 0.4

bool PhysicsServerBox2D::_body_test_motion(const RID &p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *p_result) const {
	if (!p_result) {
		return false;
	}
	Box2DBody *body = body_owner.get_or_null(p_body);
	Transform2D body_position = p_from;
	ERR_FAIL_COND_V(!body, false);
	Box2DDirectSpaceState *space_state = (Box2DDirectSpaceState *)body->get_space_state();
	ERR_FAIL_COND_V(!space_state, false);
	// 1. recover (with margin)
	{
		real_t min_contact_depth = p_margin * TEST_MOTION_MIN_CONTACT_DEPTH_FACTOR;
		for (int i = 0; i < BODY_MOTION_RECOVER_ATTEMPTS; i++) {
			Vector2 recover_step;
			Vector<b2Fixture *> query_result = Box2DSweepTest::query_aabb_motion(body->get_shapes(), body_position, Vector2(), p_margin, body->get_collision_layer(), body->get_collision_mask(), true, false, (Box2DDirectSpaceState *)body->get_space_state());
			Vector<SweepTestResult> sweep_test_results = Box2DSweepTest::multiple_shapes_cast(body->get_shapes(), body_position, Vector2(), p_margin, true, false, 64, query_result, (Box2DDirectSpaceState *)body->get_space_state());
			sweep_test_results = remove_disabled_and_one_way_wrong_direction(sweep_test_results, body, Vector2());
			if (sweep_test_results.is_empty()) {
				break;
			}
			for (SweepTestResult result : sweep_test_results) {
				Vector2 n = Vector2(result.world_manifold.normal.x, result.world_manifold.normal.y);
				// separation from margin
				if (result.distance_output.distance > min_contact_depth + CMP_EPSILON) {
					ERR_PRINT("dist margin " + rtos( result.distance_output.distance));
					recover_step -= n * ((p_margin - result.distance_output.distance) - min_contact_depth) * BODY_MOTION_RECOVER_RATIO;
				} else {
					ERR_PRINT("dist separation " + rtos(result.world_manifold.separations[0]));
					// separation from intersection
					recover_step += n * (p_margin - result.world_manifold.separations[0] - min_contact_depth) * BODY_MOTION_RECOVER_RATIO;
				}
				float dist = p_margin - (result.distance_output.pointA - result.distance_output.pointB).Length();
				Vector2 a(result.distance_output.pointA.x, result.distance_output.pointA.y);
				Vector2 b(result.distance_output.pointB.x, result.distance_output.pointB.y);
				if (result.manifold.pointCount == 2) {
					//Vector2 a(result.world_manifold.points[0].x, result.world_manifold.points[0].y);
					//Vector2 b(result.world_manifold.points[1].x, result.world_manifold.points[1].y);
				}

				// Compute plane on b towards a.
				//Vector2 n = Vector2(result.world_manifold.normal.x, result.world_manifold.normal.y);
				// Move it outside as to fit the margin
				real_t d = n.dot(b);

				// Compute depth on recovered motion.
				real_t depth = n.dot(a + recover_step) - d;
				depth = ABS(dist);
				if (depth > min_contact_depth + CMP_EPSILON) {
					// Only recover if there is penetration.
					//recover_step -= n * (depth - min_contact_depth) * BODY_MOTION_RECOVER_RATIO;
				}
			}
			body_position.columns[2] -= recover_step;
		}
	}
	// 2. find direction (without margin)
	Vector<b2Fixture *> query_result = Box2DSweepTest::query_aabb_motion(body->get_shapes(), body_position, p_motion, 0.0, body->get_collision_layer(), body->get_collision_mask(), true, false, (Box2DDirectSpaceState *)body->get_space_state());
	Vector<SweepTestResult> sweep_test_results_step_2 = Box2DSweepTest::multiple_shapes_cast(body->get_shapes(), body_position, p_motion, 0.0, true, false, 64, query_result, (Box2DDirectSpaceState *)body->get_space_state());
	sweep_test_results_step_2 = remove_disabled_and_one_way_wrong_direction(sweep_test_results_step_2, body, p_motion);
	sweep_test_results_step_2 = Box2DSweepTest::closest_result_in_cast(sweep_test_results_step_2);

	// 3. find collision (with margin)
	query_result = Box2DSweepTest::query_aabb_motion(body->get_shapes(), body_position, p_motion, p_margin, body->get_collision_layer(), body->get_collision_mask(), true, false, (Box2DDirectSpaceState *)body->get_space_state());
	Vector<SweepTestResult> sweep_test_results_step_3 = Box2DSweepTest::multiple_shapes_cast(body->get_shapes(), body_position, p_motion, p_margin, true, false, 64, query_result, (Box2DDirectSpaceState *)body->get_space_state());
	sweep_test_results_step_3 = remove_disabled_and_one_way_wrong_direction(sweep_test_results_step_3, body, p_motion);
	sweep_test_results_step_3 = Box2DSweepTest::closest_result_in_cast(sweep_test_results_step_3);

	PhysicsServer2DExtensionMotionResult &current_result = *p_result;

	current_result.travel = body_position.get_origin() - p_from.get_origin();
	if (sweep_test_results_step_3.is_empty()) {
		current_result.travel += p_motion;
		current_result.remainder = Vector2();
		current_result.collision_safe_fraction = 0;
		current_result.collision_unsafe_fraction = 0;
		return false;
	}
	SweepTestResult sweep_test_result = sweep_test_results_step_3[0];

	Box2DCollisionObject *body_B = sweep_test_result.sweep_shape_B.fixture->GetBody()->GetUserData().collision_object;

	Vector2 conveyer_belt_speed = Box2DSpaceContactListener::handle_static_constant_linear_velocity(body_B->get_b2Body(), body_B, body->get_b2Body(), body, false);

	current_result.collider = body_B->get_self();
	current_result.collider_id = body_B->get_object_instance_id();
	current_result.collision_point = box2d_to_godot(sweep_test_result.world_manifold.points[0]);
	current_result.collision_normal = -Vector2(sweep_test_result.world_manifold.normal.x, sweep_test_result.world_manifold.normal.y);
	current_result.collider_velocity = box2d_to_godot(body_B->get_b2Body()->GetLinearVelocity());
	current_result.collision_safe_fraction = sweep_test_result.safe_fraction();
	current_result.collision_unsafe_fraction = sweep_test_result.unsafe_fraction();
	if (!sweep_test_results_step_2.is_empty()) {
		current_result.collision_safe_fraction = sweep_test_results_step_2.get(0).safe_fraction();
		current_result.collision_unsafe_fraction = sweep_test_results_step_2.get(0).unsafe_fraction();
	}
	current_result.travel += p_motion * current_result.collision_safe_fraction + conveyer_belt_speed * body->get_step();
	current_result.remainder = p_motion - p_motion * current_result.collision_safe_fraction;
	int shape_A_index = 0;
	for (int i = 0; i < body->get_shape_count(); i++) {
		if (body->get_shape(i) == sweep_test_result.sweep_shape_A.shape) {
			shape_A_index = i;
		}
	}
	current_result.collision_local_shape = shape_A_index;
	current_result.collider_shape = sweep_test_result.sweep_shape_B.fixture->GetUserData().shape_idx;
	return true;
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
			WARN_PRINT_ONCE("JOINT_PARAM_BIAS is UNUSED");
			break;
		case JOINT_PARAM_MAX_BIAS:
			WARN_PRINT_ONCE("JOINT_PARAM_MAX_BIAS is UNUSED");
			break;
		case JOINT_PARAM_MAX_FORCE:
			joint->set_max_force(p_value);
			break;
		default: {
			break;
		}
	}
}
double PhysicsServerBox2D::_joint_get_param(const RID &p_joint, PhysicsServer2D::JointParam p_param) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, 0);
	switch (p_param) {
		case JOINT_PARAM_BIAS:
			WARN_PRINT_ONCE("JOINT_PARAM_BIAS is UNUSED");
			break;
		case JOINT_PARAM_MAX_BIAS:
			WARN_PRINT_ONCE("JOINT_PARAM_MAX_BIAS is UNUSED");
			break;
		case JOINT_PARAM_MAX_FORCE:
			return joint->get_max_force();
		default: {
			break;
		}
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
		case PIN_JOINT_LIMIT_UPPER: {
			joint->set_pin_upper_angle(p_value);
		} break;
		case PIN_JOINT_LIMIT_LOWER: {
			joint->set_pin_lower_angle(p_value);
		} break;
		case PIN_JOINT_MOTOR_TARGET_VELOCITY: {
			joint->set_pin_motor(p_value);
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
		case PIN_JOINT_LIMIT_UPPER: {
			return joint->get_pin_upper_angle();
		} break;
		case PIN_JOINT_LIMIT_LOWER: {
			return joint->get_pin_lower_angle();
		} break;
		case PIN_JOINT_MOTOR_TARGET_VELOCITY: {
			return joint->get_pin_motor();
		} break;
	}
	return 0;
}

void PhysicsServerBox2D::_pin_joint_set_flag(const RID &p_joint, PhysicsServer2D::PinJointFlag p_flag, bool p_enabled) {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND(!joint);
	switch (p_flag) {
		case PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED: {
			joint->set_pin_use_limits(p_enabled);
		} break;
		case PIN_JOINT_FLAG_MOTOR_ENABLED: {
			joint->set_pin_use_motor(p_enabled);
		} break;
	}
}
bool PhysicsServerBox2D::_pin_joint_get_flag(const RID &p_joint, PhysicsServer2D::PinJointFlag p_flag) const {
	Box2DJoint *joint = joint_owner.get_or_null(p_joint);
	ERR_FAIL_COND_V(!joint, false);
	switch (p_flag) {
		case PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED: {
			return joint->get_pin_softness();
		} break;
		case PIN_JOINT_FLAG_MOTOR_ENABLED: {
			return joint->get_pin_use_motor();
		} break;
	}
	return false;
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
		if (shape) {
			shape_owner.free(p_rid);
			memdelete(shape);
		}
	} else if (area_owner.owns(p_rid)) {
		Box2DArea *area = area_owner.get_or_null(p_rid);
		if (area) {
			area_set_space(p_rid, RID());
			area_clear_shapes(p_rid);
			area_owner.free(p_rid);
			memdelete(area);
		}
	} else if (body_owner.owns(p_rid)) {
		Box2DBody *body = body_owner.get_or_null(p_rid);
		if (body) {
			body_set_space(p_rid, RID());
			body_clear_shapes(p_rid);
			body_owner.free(p_rid);
			memdelete(body);
		}
	} else if (space_owner.owns(p_rid)) {
		Box2DSpace *space = space_owner.get_or_null(p_rid);
		if (space) {
			active_spaces.erase(space);
			space_owner.free(p_rid);
			memdelete(space);
		}
	} else if (joint_owner.owns(p_rid)) {
		Box2DJoint *joint = joint_owner.get_or_null(p_rid);
		if (joint) {
			joint_owner.free(p_rid);
			memdelete(joint);
		}
	}
}

void PhysicsServerBox2D::_set_active(bool p_active) {
	active = p_active;
}

void PhysicsServerBox2D::_init() {
}

void PhysicsServerBox2D::_step(double p_step) {
	step_amount = p_step;
	if (!active) {
		return;
	}

	for (const Box2DSpace *E : active_spaces) {
		Box2DSpace *space = const_cast<Box2DSpace *>(E);
		space->step((double)p_step);
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
}

PhysicsServerBox2D::~PhysicsServerBox2D() {
}
