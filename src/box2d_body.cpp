#include "box2d_body.h"

void Box2DBody::set_space(Box2DSpace *p_space) {
    // TODO: clean up previous space
    _set_space(p_space);
    // TODO: inform new space
}

Box2DBody::Box2DBody() {
}

Box2DBody::~Box2DBody() {
}
