#include "box2d_shape_2d.h"

void Box2DShape2D::configure(const Rect2 &p_aabb) {
	aabb = p_aabb;
	configured = true;
	for (const KeyValue<Box2DShapeOwner2D *, int> &E : owners) {
		Box2DShapeOwner2D *co = const_cast<Box2DShapeOwner2D *>(E.key);
		co->_shape_changed(this);
	}
}

void Box2DShape2D::destroy_box2d_shape() {
	if (box2d::is_handle_valid(shape_handle)) {
		box2d::shape_destroy(shape_handle);
		shape_handle = box2d::invalid_shape_handle();
	}
}

b2Shape* Box2DShape2D::get_box2d_shape() {
	if (!box2d::is_handle_valid(shape_handle)) {
		shape_handle = create_box2d_shape();
	}

	return shape_handle;
}

void Box2DShape2D::add_owner(Box2DShapeOwner2D *p_owner) {
	HashMap<Box2DShapeOwner2D *, int>::Iterator E = owners.find(p_owner);
	if (E) {
		E->value++;
	} else {
		owners[p_owner] = 1;
	}
}

void Box2DShape2D::remove_owner(Box2DShapeOwner2D *p_owner) {
	HashMap<Box2DShapeOwner2D *, int>::Iterator E = owners.find(p_owner);
	ERR_FAIL_COND(!E);
	E->value--;
	if (E->value == 0) {
		owners.remove(E);
	}
}

bool Box2DShape2D::is_owner(Box2DShapeOwner2D *p_owner) const {
	return owners.has(p_owner);
}

const HashMap<Box2DShapeOwner2D *, int> &Box2DShape2D::get_owners() const {
	return owners;
}

Box2DShape2D::~Box2DShape2D() {
	ERR_FAIL_COND(owners.size());
	destroy_box2d_shape();
}
