#include "box2d_body.h"
#include "box2d_direct_body_state.h"

void Box2DBody::set_state_sync_callback(void *p_instance, BodyStateCallback p_callback) {
	body_state_callback_instance = p_instance;
	body_state_callback = p_callback;
}

Box2DDirectBodyState *Box2DBody::get_direct_state() {
	if (!direct_state) {
		direct_state = memnew(Box2DDirectBodyState);
		direct_state->body = this;
	}
	return direct_state;
}

void Box2DBody::set_active(bool p_active) {
	if (active == p_active) {
		return;
	}

	active = p_active;

	if (active) {
		if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
			// Static bodies can't be active.
			active = false;
		} else if (get_space()) {
			get_space()->body_add_to_active_list(&active_list);
		}
	} else if (get_space()) {
		get_space()->body_remove_from_active_list(&active_list);
	}
}

void Box2DBody::set_mode(PhysicsServer2D::BodyMode p_mode) {
	PhysicsServer2D::BodyMode prev = mode;
	mode = p_mode;

	switch (p_mode) {
		case PhysicsServer2D::BODY_MODE_RIGID: {
			// TODO: (inverse) mass calculation?
			//_set_static(false);
			set_active(true);
		} break;
		// TODO: other cases
	}
}

PhysicsServer2D::BodyMode Box2DBody::get_mode() const {
	return mode;
}

void Box2DBody::set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			// TODO: handle different body modes
			if (true) // rigid body
			{
				Transform2D t = p_variant;
				t.orthonormalize();
				new_transform = get_transform(); // used as old to compute motion
				if (t == new_transform) {
					break;
				}
				_set_transform(t);
				//_set_inv_transform(get_transform().inverse());
				//_update_transform_dependent();
			}
			wakeup();
		} break;
		// TODO: other cases
	}
}

Variant Box2DBody::get_state(PhysicsServer2D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			return get_transform();
		}
		// TODO: other cases
	}
	return Variant();
}

void Box2DBody::set_space(Box2DSpace *p_space) {
	if (get_space()) {
		// TODO: clean up more
		if (active_list.in_list()) {
			get_space()->body_remove_from_active_list(&active_list);
		}
	}

	_set_space(p_space);

	if (get_space()) {
		// TODO: do more
		if (active) {
			get_space()->body_add_to_active_list(&active_list);
		}
	}
}

Box2DBody::Box2DBody() : active_list(this) {
}

Box2DBody::~Box2DBody() {
}
