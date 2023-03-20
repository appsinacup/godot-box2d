#include "box2d_direct_body_state.h"

void Box2DDirectBodyState::_set_linear_velocity(const Vector2 &p_velocity) {
	ERR_FAIL_NULL(body);
	body->wakeup();
	body->set_linear_velocity(p_velocity);
}

Vector2 Box2DDirectBodyState::_get_linear_velocity() const {
	ERR_FAIL_NULL_V(body, Vector2());
	return body->get_linear_velocity();
}

void Box2DDirectBodyState::_set_angular_velocity(double p_velocity) { // should be real_t
	ERR_FAIL_NULL(body);
	body->wakeup();
	body->set_angular_velocity((real_t)p_velocity);
}

double Box2DDirectBodyState::_get_angular_velocity() const { // should be real_t
	ERR_FAIL_NULL_V(body, 0.0);
	return (double)body->get_angular_velocity();
}

void Box2DDirectBodyState::_set_transform(const Transform2D &p_transform) {
	ERR_FAIL_NULL(body);
	body->set_state(PhysicsServer2D::BODY_STATE_TRANSFORM, p_transform);
}

Transform2D Box2DDirectBodyState::_get_transform() const {
	ERR_FAIL_NULL_V(body, Transform2D());
	return body->get_transform();
}
