#include "box2d_direct_body_state.h"

Vector2 Box2DDirectBodyState::_get_total_gravity() const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_total_gravity();
}

double Box2DDirectBodyState::_get_total_linear_damp() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_total_linear_damp();
}

double Box2DDirectBodyState::_get_total_angular_damp() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_total_angular_damp();
}

Vector2 Box2DDirectBodyState::_get_center_of_mass() const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_center_of_mass();
}

Vector2 Box2DDirectBodyState::_get_center_of_mass_local() const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_center_of_mass_local();
}

double Box2DDirectBodyState::_get_inverse_mass() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_inverse_mass();
}
double Box2DDirectBodyState::_get_inverse_inertia() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_inverse_inertia();
}
void Box2DDirectBodyState::_set_linear_velocity(const Vector2 &velocity) {
	ERR_FAIL_NULL(body);
	body->set_linear_velocity(velocity);
}
Vector2 Box2DDirectBodyState::_get_linear_velocity() const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_linear_velocity();
}
void Box2DDirectBodyState::_set_angular_velocity(double velocity) {
	ERR_FAIL_NULL(body);
	body->set_angular_velocity(velocity);
}
double Box2DDirectBodyState::_get_angular_velocity() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_angular_velocity();
}
void Box2DDirectBodyState::_set_transform(const Transform2D &transform) {
	ERR_FAIL_NULL(body);
	body->set_transform(transform);
}
Transform2D Box2DDirectBodyState::_get_transform() const {
	ERR_FAIL_NULL_V(body, Transform2D());
	return body->get_transform();
}
Vector2 Box2DDirectBodyState::_get_velocity_at_local_position(const Vector2 &local_position) const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_velocity_at_local_position(local_position);
}
void Box2DDirectBodyState::_apply_central_impulse(const Vector2 &impulse) {
	ERR_FAIL_NULL(body);
	body->apply_central_impulse(impulse);
}
void Box2DDirectBodyState::_apply_impulse(const Vector2 &impulse, const Vector2 &position) {
	ERR_FAIL_NULL(body);
	body->apply_impulse(impulse, position);
}
void Box2DDirectBodyState::_apply_torque_impulse(double impulse) {
	ERR_FAIL_NULL(body);
	body->apply_torque_impulse(impulse);
}
void Box2DDirectBodyState::_apply_central_force(const Vector2 &force) {
	ERR_FAIL_NULL(body);
	body->apply_central_force(force);
}
void Box2DDirectBodyState::_apply_force(const Vector2 &force, const Vector2 &position) {
	ERR_FAIL_NULL(body);
	body->apply_force(force, position);
}
void Box2DDirectBodyState::_apply_torque(double torque) {
	ERR_FAIL_NULL(body);
	body->apply_torque(torque);
}
void Box2DDirectBodyState::_add_constant_central_force(const Vector2 &force) {
	ERR_FAIL_NULL(body);
	body->add_constant_central_force(force);
}
void Box2DDirectBodyState::_add_constant_force(const Vector2 &force, const Vector2 &position) {
	ERR_FAIL_NULL(body);
	body->add_constant_force(force, position);
}
void Box2DDirectBodyState::_add_constant_torque(double torque) {
	ERR_FAIL_NULL(body);
	body->add_constant_torque(torque);
}
void Box2DDirectBodyState::_set_constant_force(const Vector2 &force) {
	ERR_FAIL_NULL(body);
	body->set_constant_force(force);
}
Vector2 Box2DDirectBodyState::_get_constant_force() const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_constant_force();
}
void Box2DDirectBodyState::_set_constant_torque(double torque) {
	ERR_FAIL_NULL(body);
	body->set_constant_torque(torque);
}
double Box2DDirectBodyState::_get_constant_torque() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_constant_torque();
}
void Box2DDirectBodyState::_set_sleep_state(bool enabled) {
	ERR_FAIL_NULL(body);
	body->set_sleep_state(enabled);
}
bool Box2DDirectBodyState::_is_sleeping() const {
	ERR_FAIL_NULL_V(body, false);
	return body->is_sleeping();
}
int32_t Box2DDirectBodyState::_get_contact_count() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_contact_count();
}
Vector2 Box2DDirectBodyState::_get_contact_local_position(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_contact_local_position(contact_idx);
}
Vector2 Box2DDirectBodyState::_get_contact_local_normal(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_contact_local_normal(contact_idx);
}
int32_t Box2DDirectBodyState::_get_contact_local_shape(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_contact_local_shape(contact_idx);
}
RID Box2DDirectBodyState::_get_contact_collider(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, RID());
	return body->get_contact_collider(contact_idx);
}
Vector2 Box2DDirectBodyState::_get_contact_collider_position(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_contact_collider_position(contact_idx);
}
uint64_t Box2DDirectBodyState::_get_contact_collider_id(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_contact_collider_id(contact_idx);
}
Object *Box2DDirectBodyState::_get_contact_collider_object(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, nullptr);
	return body->get_contact_collider_object(contact_idx);
}
int32_t Box2DDirectBodyState::_get_contact_collider_shape(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_contact_collider_shape(contact_idx);
}
Vector2 Box2DDirectBodyState::_get_contact_collider_velocity_at_position(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_contact_collider_velocity_at_position(contact_idx);
}
Vector2 Box2DDirectBodyState::_get_contact_impulse(int32_t contact_idx) const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_contact_impulse(contact_idx);
}
double Box2DDirectBodyState::_get_step() const {
	ERR_FAIL_NULL_V(body, 0);
	return body->get_step();
}
void Box2DDirectBodyState::_integrate_forces() {
	ERR_FAIL_NULL(body);
	body->integrate_forces();
}
PhysicsDirectSpaceState2D *Box2DDirectBodyState::_get_space_state() {
	ERR_FAIL_NULL_V(body, nullptr);
	return body->get_space_state();
}
