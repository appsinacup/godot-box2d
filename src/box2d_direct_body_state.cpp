#include "box2d_direct_body_state.h"

void Box2DDirectBodyState::_set_transform(const Transform2D &p_transform) {
	body->set_state(PhysicsServer2D::BODY_STATE_TRANSFORM, p_transform);
}

Transform2D Box2DDirectBodyState::_get_transform() const {
	return body->get_transform();
}

