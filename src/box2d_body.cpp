#include "box2d_body.h"
#include "box2d_direct_body_state.h"

void Box2DBody::set_state_sync_callback(const Callable &p_callable) {
	body_state_callback = p_callable;
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

	b2BodyDef* body_def = memnew(b2BodyDef);

	switch (p_mode) {
		case PhysicsServer2D::BODY_MODE_STATIC: {
			// TODO: other stuff
			body_def->type = b2_staticBody;
			set_active(false);
		} break;
		case PhysicsServer2D::BODY_MODE_KINEMATIC: {
			// TODO: other stuff
			body_def->type = b2_kinematicBody;
			set_active(true); // TODO: consider contacts
		} break;
		case PhysicsServer2D::BODY_MODE_RIGID:
		case PhysicsServer2D::BODY_MODE_RIGID_LINEAR: {
			// TODO: (inverse) mass calculation?
			//_set_static(false);
			body_def->type = b2_dynamicBody;
			set_active(true);
		} break;
	}

	set_b2BodyDef(body_def);
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
		if (direct_state_query_list.in_list()) {
			get_space()->body_remove_from_state_query_list(&direct_state_query_list);
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

void Box2DBody::after_step() {
	_set_transform_from_box2d();
	if (body_state_callback.is_valid()) {
		get_space()->body_add_to_state_query_list(&direct_state_query_list);
	}
}

void Box2DBody::call_queries() {
	Variant direct_state = get_direct_state();
	if (body_state_callback.is_valid()) {
		body_state_callback.callv(Array::make(direct_state));
	}
}

Box2DBody::Box2DBody() : active_list(this), direct_state_query_list(this) {
}

Box2DBody::~Box2DBody() {
	if (direct_state) {
		memdelete(direct_state);
	}
}
