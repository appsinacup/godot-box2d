#include "box2d_body.h"
#include "../box2d_type_conversions.h"
#include "box2d_direct_body_state.h"

bool Box2DBody::is_active() const { return active; }

// Physics Server

void Box2DBody::set_max_contacts_reported(int32 p_max_contacts_reported) {
	max_contacts_reported = p_max_contacts_reported;
}

int32 Box2DBody::get_max_contacts_reported() {
	return max_contacts_reported;
}

void Box2DBody::wakeup() {
	if ((!get_space()) || mode == PhysicsServer2D::BODY_MODE_STATIC || mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
		return;
	}
	set_active(true);
}

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
			set_sleep_state(true);
			get_space()->body_add_to_active_list(&active_list);
		}
	} else if (get_space()) {
		set_sleep_state(false);
		get_space()->body_remove_from_active_list(&active_list);
	}
}

void Box2DBody::set_mode(PhysicsServer2D::BodyMode p_mode) {
	if (mode == p_mode) {
		return;
	}
	mode = p_mode;
	switch (p_mode) {
		case PhysicsServer2D::BODY_MODE_STATIC: {
			// TODO: other stuff
			body_def->type = b2_staticBody;
			body_def->fixedRotation = false;
			set_active(false);
		} break;
		case PhysicsServer2D::BODY_MODE_KINEMATIC: {
			// TODO: other stuff
			body_def->type = b2_kinematicBody;
			body_def->fixedRotation = false;
			set_active(true); // TODO: consider contacts
		} break;
		case PhysicsServer2D::BODY_MODE_RIGID: {
			body_def->type = b2_dynamicBody;
			body_def->fixedRotation = false;
			set_active(true);
		} break;
		case PhysicsServer2D::BODY_MODE_RIGID_LINEAR: {
			// TODO: (inverse) mass calculation?
			//_set_static(false);
			body_def->type = b2_dynamicBody;
			body_def->fixedRotation = true;
			set_active(true);
		} break;
	}
	if (body) {
		body->SetType(body_def->type);
		body->SetFixedRotation(body_def->fixedRotation);
		body->SetMassData(&mass_data);
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
			} else if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
				_set_transform(p_variant);
				//_set_inv_transform(get_transform().affine_inverse());
				//wakeup_neighbours();
			} else { // rigid body
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
			float angular_velocity = godot_to_box2d(variant_to_number(p_variant));
			set_angular_velocity(angular_velocity);
			wakeup();
		} break;
		case PhysicsServer2D::BODY_STATE_SLEEPING: {
			if (mode == PhysicsServer2D::BODY_MODE_STATIC || mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
				break;
			}
			bool do_sleep = p_variant;
			if (do_sleep) {
				set_linear_velocity(Vector2());
				set_angular_velocity(0);
				set_active(false);
			} else {
				if (mode != PhysicsServer2D::BODY_MODE_STATIC) {
					set_active(true);
				}
			}
		} break;
		case PhysicsServer2D::BODY_STATE_CAN_SLEEP: {
			can_sleep = p_variant;
			if (body) {
				body->SetSleepingAllowed(can_sleep);
			}
			if (mode >= PhysicsServer2D::BODY_MODE_RIGID && !active && !can_sleep) {
				set_active(true);
			}
		} break;
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
		case PhysicsServer2D::BODY_STATE_SLEEPING: {
			return !is_active();
		}
		case PhysicsServer2D::BODY_STATE_CAN_SLEEP: {
			return can_sleep;
		}
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
		if (body) {
			body->SetAwake(active);
		}
		if (active) {
			get_space()->body_add_to_active_list(&active_list);
		}
	}
}

void Box2DBody::after_step() {
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

void Box2DBody::set_continuous_collision_detection_mode(PhysicsServer2D::CCDMode p_mode) {
	if (collision_mode == p_mode) {
		return;
	}
	collision_mode = p_mode;
	switch (collision_mode) {
		case PhysicsServer2D::CCD_MODE_DISABLED: {
			body_def->bullet = false;
		} break;
		case PhysicsServer2D::CCD_MODE_CAST_RAY:
		case PhysicsServer2D::CCD_MODE_CAST_SHAPE:
			body_def->bullet = true;
			break;
	}
	if (body) {
		body->SetBullet(body_def->bullet);
	}
}
PhysicsServer2D::CCDMode Box2DBody::get_continuous_collision_detection_mode() const {
	return collision_mode;
}

void Box2DBody::add_joint(Box2DJoint *p_joint) {
	joints.insert(p_joint);
}
void Box2DBody::remove_joint(Box2DJoint *p_joint) {
	joints.erase(p_joint);
}

HashSet<Box2DJoint *> Box2DBody::get_joints() {
	return joints;
}

Box2DBody::Box2DBody() :
		Box2DCollisionObject(TYPE_BODY),
		active_list(this),
		direct_state_query_list(this) {
}

Box2DBody::~Box2DBody() {
	if (direct_state) {
		memdelete(direct_state);
	}
}
