#include "box2d_direct_body_state.h"

void Box2DDirectBodyState::_set_linear_velocity(const Vector2 &p_velocity) {
	body->wakeup();
	body->set_linear_velocity(p_velocity);
}

Vector2 Box2DDirectBodyState::_get_linear_velocity() const {
	return body->get_linear_velocity();
}

void Box2DDirectBodyState::_set_angular_velocity(double p_velocity) { // should be real_t
	body->wakeup();
	body->set_angular_velocity((real_t)p_velocity);
}

double Box2DDirectBodyState::_get_angular_velocity() const { // should be real_t
	return (double)body->get_angular_velocity();
}

void Box2DDirectBodyState::_set_transform(const Transform2D &p_transform) {
	body->set_state(PhysicsServer2D::BODY_STATE_TRANSFORM, p_transform);
}

Transform2D Box2DDirectBodyState::_get_transform() const {
	return body->get_transform();
}

