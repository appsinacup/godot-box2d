#include "box2d_area.h"

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

Box2DArea::Box2DArea() :
		Box2DCollisionObject(TYPE_AREA) {
	//_set_static(true); //areas are not active by default
}

Box2DArea::~Box2DArea() {
}
