#include "box2d_area.h"
#include "box2d_body.h"

// Physics Server
void Box2DArea::set_monitorable(bool p_monitorable) {
	monitorable = p_monitorable;
}
bool Box2DArea::get_monitorable() {
	return monitorable;
}
bool Box2DArea::get_monitoring() {
	return area_monitor_callback.is_valid();
}
void Box2DArea::set_monitor_callback(const Callable &p_callback) {
	monitor_callback = p_callback;
}
void Box2DArea::set_area_monitor_callback(const Callable &p_callback) {
	area_monitor_callback = p_callback;
}
void Box2DArea::call_area_monitor(Box2DArea *area, PhysicsServer2D::AreaBodyStatus status, const RID &p_area, ObjectID p_instance, int area_shape_idx, int self_shape_idx) {
	if (get_monitoring() && area->monitorable) {
		area_monitor_callback.callv(Array::make(status, p_area, p_instance, area_shape_idx, self_shape_idx));
	}
}
void Box2DArea::call_monitor(Box2DCollisionObject *body, PhysicsServer2D::AreaBodyStatus status, const RID &p_body, ObjectID p_instance, int32_t area_shape_idx, int32_t self_shape_idx) {
	if (monitor_callback.is_valid()) {
		monitor_callback.callv(Array::make(status, p_body, p_instance, area_shape_idx, self_shape_idx));
		if (status == PhysicsServer2D::AreaBodyStatus::AREA_BODY_ADDED) {
			add_body(body);
		} else {
			remove_body(body);
		}
	}
}

void Box2DArea::set_transform(const Transform2D &p_transform) {
	// TODO: add to moved list?

	_set_transform(p_transform);
	// _set_inv_transform(p_transform.affine_inverse());
}

void Box2DArea::set_space(Box2DSpace *p_space) {
	// TODO: remove from monitor query list, remove from moved list?

	//monitored_bodies.clear();
	//monitored_areas.clear();

	_set_space(p_space);
}

void Box2DArea::set_priority(real_t p_priority) {
	if (collision.priority == p_priority) {
		return;
	}
	collision.priority = p_priority;
	for (Box2DCollisionObject *body : bodies) {
		body->sort_areas();
		body->recalculate_total_gravity();
		body->recalculate_total_angular_damp();
		body->recalculate_total_linear_damp();
	}
}

void Box2DArea::set_gravity_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	if (gravity.override_mode == p_value) {
		return;
	}
	gravity.override_mode = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_gravity(real_t p_value) {
	if (gravity.gravity == godot_to_box2d(p_value)) {
		return;
	}
	gravity.gravity = godot_to_box2d(p_value);
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_gravity_vector(Vector2 p_value) {
	if (gravity.vector == b2Vec2(p_value.x, p_value.y)) {
		return;
	}
	gravity.vector = b2Vec2(p_value.x, p_value.y);
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_gravity_is_point(bool p_value) {
	if (gravity.is_point == p_value) {
		return;
	}
	gravity.is_point = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_gravity_point_unit_distance(double p_value) {
	if (gravity.point_unit_distance == p_value) {
		return;
	}
	gravity.point_unit_distance = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_gravity();
	}
}
void Box2DArea::set_linear_damp_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	if (linear_damp_override_mode == p_value) {
		return;
	}
	linear_damp_override_mode = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_linear_damp();
	}
}
void Box2DArea::set_angular_damp_override_mode(PhysicsServer2D::AreaSpaceOverrideMode p_value) {
	if (angular_damp_override_mode == p_value) {
		return;
	}
	angular_damp_override_mode = p_value;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_angular_damp();
	}
}

void Box2DArea::set_linear_damp(real_t p_linear_damp) {
	if (damping.linear_damp == p_linear_damp) {
		return;
	}
	damping.linear_damp = p_linear_damp;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_linear_damp();
	}
}
void Box2DArea::set_angular_damp(real_t p_angular_damp) {
	if (damping.angular_damp == p_angular_damp) {
		return;
	}
	damping.angular_damp = p_angular_damp;
	for (Box2DCollisionObject *body : bodies) {
		body->recalculate_total_angular_damp();
	}
}

PhysicsServer2D::AreaSpaceOverrideMode Box2DArea::get_gravity_override_mode() const {
	return gravity.override_mode;
}
double Box2DArea::get_gravity() const {
	return box2d_to_godot(gravity.gravity);
}

b2Vec2 Box2DArea::get_b2_gravity(Transform2D transform) const {
	if (!gravity.is_point) {
		return gravity.gravity * gravity.vector;
	}

	const Vector2 point = get_transform().xform(box2d_to_godot(gravity.vector));
	const Vector2 to_point = point - transform.get_origin();
	const float to_point_dist_sq = std::max(to_point.length_squared(), CMP_EPSILON);
	const Vector2 to_point_dir = to_point / Math::sqrt(to_point_dist_sq);

	const float gravity_dist_sq = gravity.point_unit_distance * gravity.point_unit_distance;

	return godot_to_box2d((to_point_dir * (get_gravity() * gravity_dist_sq / to_point_dist_sq)));
}
Vector2 Box2DArea::get_gravity_vector() const {
	return Vector2(gravity.vector.x, gravity.vector.y);
}
bool Box2DArea::get_gravity_is_point() const {
	return gravity.is_point;
}
double Box2DArea::get_gravity_point_unit_distance() const {
	return gravity.point_unit_distance;
}
PhysicsServer2D::AreaSpaceOverrideMode Box2DArea::get_linear_damp_override_mode() const {
	return linear_damp_override_mode;
}
PhysicsServer2D::AreaSpaceOverrideMode Box2DArea::get_angular_damp_override_mode() const {
	return angular_damp_override_mode;
}
void Box2DArea::add_body(Box2DCollisionObject *p_body) {
	bodies.append(p_body);
	p_body->add_area(this);
}
void Box2DArea::remove_body(Box2DCollisionObject *p_body) {
	bodies.erase(p_body);
	p_body->remove_area(this);
}

Box2DArea::Box2DArea() :
		Box2DCollisionObject(TYPE_AREA) {
	damping.linear_damp = 0.1;
	damping.angular_damp = 1;
	// areas are sensors and dynamic bodies, but don't move.
	// b2_staticBody don't collide with b2_staticBody or b2_kinematicBody
	// b2_kinematicBody don't collide with b2_staticBody or b2_kinematicBody
	// and areas have to be able to intersect with both kinematic and static bodies
	body_def->type = b2_dynamicBody;
	//_set_static(true); //areas are not active by default
}

Box2DArea::~Box2DArea() {
	for (Box2DCollisionObject *body : bodies) {
		if (body) {
			body->remove_area(this);
		}
	}
}
