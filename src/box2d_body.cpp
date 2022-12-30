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

void Box2DBody::set_linear_velocity(const Vector2 &p_linear_velocity) {
	b2Vec2 box2d_linear_velocity;
	godot_to_box2d(p_linear_velocity, box2d_linear_velocity);
	body->SetLinearVelocity(box2d_linear_velocity);
}

Vector2 Box2DBody::get_linear_velocity() const {
	b2Vec2 box2d_linear_velocity = body->GetLinearVelocity();
	Vector2 linear_velocity;
	box2d_to_godot(box2d_linear_velocity, linear_velocity);
	return linear_velocity;
}

void Box2DBody::set_angular_velocity(real_t p_angular_velocity) {
	float box2d_angular_velocity;
	godot_to_box2d(p_angular_velocity, box2d_angular_velocity);
	body->SetAngularVelocity(box2d_angular_velocity);
}

real_t Box2DBody::get_angular_velocity() const {
	float box2d_angular_velocity = body->GetAngularVelocity();
	real_t angular_velocity;
	box2d_to_godot(box2d_angular_velocity, angular_velocity);
	return angular_velocity;
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
}

PhysicsServer2D::BodyMode Box2DBody::get_mode() const {
	return mode;
}

void Box2DBody::set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant) {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			if (mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
				// TODO
			}
			else if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
				_set_transform(p_variant);
				//_set_inv_transform(get_transform().affine_inverse());
				//wakeup_neighbours();
			}
			else // rigid body
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
		case PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY: {
			Vector2 linear_velocity = p_variant;
			set_linear_velocity(linear_velocity);
			wakeup();
		} break;
		case PhysicsServer2D::BODY_STATE_ANGULAR_VELOCITY: {
			float angular_velocity = p_variant;
			set_angular_velocity(angular_velocity);
			wakeup();
		} break;
		// TODO: other cases
	}
}

Variant Box2DBody::get_state(PhysicsServer2D::BodyState p_state) const {
	switch (p_state) {
		case PhysicsServer2D::BODY_STATE_TRANSFORM: {
			return get_transform();
		} break;
		case PhysicsServer2D::BODY_STATE_LINEAR_VELOCITY: {
			return get_linear_velocity();
		} break;
		case PhysicsServer2D::BODY_STATE_ANGULAR_VELOCITY: {
			return get_angular_velocity();
		} break;
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
