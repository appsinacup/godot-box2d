#include "box2d_space_2d.h"
#include "box2d_direct_space_state_2d.h"

#include "../servers/box2d_body_utils_2d.h"
#include "../servers/box2d_physics_server_2d.h"
#include "../servers/box2d_project_settings.h"

#include <godot_cpp/classes/project_settings.hpp>

#define TEST_MOTION_MARGIN_MIN_VALUE 0.0001

void Box2DSpace2D::body_add_to_active_list(SelfList<Box2DBody2D> *p_body) {
	active_list.add(p_body);
}

void Box2DSpace2D::body_add_to_mass_properties_update_list(SelfList<Box2DBody2D> *p_body) {
	mass_properties_update_list.add(p_body);
}

void Box2DSpace2D::body_add_to_gravity_update_list(SelfList<Box2DBody2D> *p_body) {
	gravity_update_list.add(p_body);
}

void Box2DSpace2D::body_add_to_state_query_list(SelfList<Box2DBody2D> *p_body) {
	state_query_list.add(p_body);
}

void Box2DSpace2D::area_add_to_monitor_query_list(SelfList<Box2DArea2D> *p_area) {
	monitor_query_list.add(p_area);
}

void Box2DSpace2D::area_add_to_area_update_list(SelfList<Box2DArea2D> *p_area) {
	area_update_list.add(p_area);
}

void Box2DSpace2D::body_add_to_area_update_list(SelfList<Box2DBody2D> *p_body) {
	body_area_update_list.add(p_body);
}

void Box2DSpace2D::add_removed_collider(b2Fixture *p_handle, Box2DCollisionObject2D *p_object, uint32_t p_shape_index) {
	uint64_t handle_hash = box2d::handle_hash(p_handle);
	ERR_FAIL_COND(removed_colliders.has(handle_hash));
	removed_colliders.insert(handle_hash, { p_object->get_rid(), p_object->get_instance_id(), p_shape_index, p_object->get_type() });
}

bool Box2DSpace2D::get_removed_collider_info(b2Fixture *p_handle, RID &r_rid, ObjectID &r_instance_id, uint32_t &r_shape_index, Box2DCollisionObject2D::Type &r_type) const {
	uint64_t handle_hash = box2d::handle_hash(p_handle);
	auto foundIt = removed_colliders.find(handle_hash);
	if (foundIt == removed_colliders.end()) {
		return false;
	}

	r_rid = foundIt->value.rid;
	r_instance_id = foundIt->value.instance_id;
	r_shape_index = foundIt->value.shape_index;
	r_type = foundIt->value.type;
	return true;
}

void Box2DSpace2D::active_body_callback(const box2d::ActiveBodyInfo &active_body_info) {
	Box2DCollisionObject2D *pObject = nullptr;
	if (box2d::is_user_data_valid(active_body_info.body_user_data)) {
		pObject = Box2DCollisionObject2D::get_body_user_data(active_body_info.body_user_data);
	}

	ERR_FAIL_COND(!pObject);
	ERR_FAIL_COND(pObject->get_type() != Box2DCollisionObject2D::TYPE_BODY);

	Box2DBody2D *pBody = static_cast<Box2DBody2D *>(pObject);
	pBody->on_marked_active();
}

bool Box2DSpace2D::collision_filter_common_callback(b2World *world_handle, const box2d::CollisionFilterInfo *filter_info, CollidersInfo &r_colliders_info) {
	Box2DSpace2D *space = Box2DPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND_V(!space, false);

	if (box2d::is_user_data_valid(filter_info->user_data1)) {
		r_colliders_info.object1 = Box2DCollisionObject2D::get_collider_user_data(filter_info->user_data1, r_colliders_info.shape1);
	}

	if (box2d::is_user_data_valid(filter_info->user_data2)) {
		r_colliders_info.object2 = Box2DCollisionObject2D::get_collider_user_data(filter_info->user_data2, r_colliders_info.shape2);
	}

	ERR_FAIL_COND_V(!r_colliders_info.object1, false);
	ERR_FAIL_COND_V(!r_colliders_info.object2, false);

	if (!r_colliders_info.object1->interacts_with(r_colliders_info.object2)) {
		return false;
	}

	return true;
}

bool Box2DSpace2D::ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) {
	ERR_FAIL_COND_V(!box2d::is_handle_valid(fixtureA), false);
	ERR_FAIL_COND_V(!box2d::is_handle_valid(fixtureB), false);
	b2FixtureUserData user_dataA = fixtureA->GetUserData();
	b2FixtureUserData user_dataB = fixtureB->GetUserData();
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(user_dataA), false);
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(user_dataB), false);
	CollidersInfo colliders_info;
	box2d::CollisionFilterInfo filter_info;
	filter_info.user_data1 = user_dataA;
	filter_info.user_data2 = user_dataB;
	if (!collision_filter_common_callback(handle, &filter_info, colliders_info)) {
		return false;
	}
	if (colliders_info.object1->get_type() == Box2DCollisionObject2D::TYPE_BODY &&
			colliders_info.object2->get_type() == Box2DCollisionObject2D::TYPE_BODY) {
		const Box2DBody2D *body1 = static_cast<const Box2DBody2D *>(colliders_info.object1);
		const Box2DBody2D *body2 = static_cast<const Box2DBody2D *>(colliders_info.object2);
		if (body1->has_exception(body2->get_rid()) || body2->has_exception(body1->get_rid())) {
			return false;
		}
	}

	return true;
}
box2d::CollisionEventInfo event_info_from_contact(b2Contact *contact) {
	box2d::CollisionEventInfo event_info;
	event_info.is_sensor = false;
	event_info.is_valid = false;
	event_info.collider1 = contact->GetFixtureA();
	event_info.collider2 = contact->GetFixtureB();
	ERR_FAIL_COND_V(!box2d::is_handle_valid(event_info.collider1), event_info);
	ERR_FAIL_COND_V(!box2d::is_handle_valid(event_info.collider2), event_info);
	event_info.user_data1 = event_info.collider1->GetUserData();
	event_info.user_data2 = event_info.collider2->GetUserData();
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(event_info.user_data1), event_info);
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(event_info.user_data2), event_info);
	event_info.is_sensor = event_info.collider1->IsSensor() || event_info.collider2->IsSensor();
	event_info.is_valid = event_info.is_sensor;
	return event_info;
}
box2d::CollisionFilterInfo filter_info_from_contact(b2Contact *contact) {
	box2d::CollisionFilterInfo event_info;
	event_info.is_valid = false;
	ERR_FAIL_COND_V(!box2d::is_handle_valid(contact->GetFixtureA()), event_info);
	ERR_FAIL_COND_V(!box2d::is_handle_valid(contact->GetFixtureB()), event_info);
	event_info.user_data1 = contact->GetFixtureA()->GetUserData();
	event_info.user_data2 = contact->GetFixtureB()->GetUserData();
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(event_info.user_data1), event_info);
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(event_info.user_data2), event_info);
	event_info.is_valid = true;
	return event_info;
}
box2d::ContactForceEventInfo force_info_from_contact(b2Contact *contact) {
	box2d::ContactForceEventInfo event_info;
	event_info.is_valid = false;
	event_info.collider1 = contact->GetFixtureA();
	event_info.collider2 = contact->GetFixtureB();
	ERR_FAIL_COND_V(!box2d::is_handle_valid(event_info.collider1), event_info);
	ERR_FAIL_COND_V(!box2d::is_handle_valid(event_info.collider2), event_info);
	event_info.user_data1 = event_info.collider1->GetUserData();
	event_info.user_data2 = event_info.collider2->GetUserData();
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(event_info.user_data1), event_info);
	ERR_FAIL_COND_V(!box2d::is_user_data_valid(event_info.user_data2), event_info);
	event_info.is_valid = true;
	return event_info;
}
void Box2DSpace2D::BeginContact(b2Contact *contact) {
	box2d::CollisionEventInfo event_info = event_info_from_contact(contact);
	if (!event_info.is_valid) {
		return;
	}
	event_info.is_started = true;
	collision_event_callback(handle, &event_info);
}

void Box2DSpace2D::EndContact(b2Contact *contact) {
	box2d::CollisionEventInfo event_info = event_info_from_contact(contact);
	if (!event_info.is_valid) {
		return;
	}
	event_info.is_started = false;
	collision_event_callback(handle, &event_info);
}
void Box2DSpace2D::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
	uint32_t shape1;
	uint32_t shape2;
	Box2DCollisionObject2D *collision_object_1 = Box2DCollisionObject2D::get_collider_user_data(contact->GetFixtureA()->GetUserData(), shape1);
	Box2DCollisionObject2D *collision_object_2 = Box2DCollisionObject2D::get_collider_user_data(contact->GetFixtureB()->GetUserData(), shape2);
	ERR_FAIL_COND(!collision_object_1);
	ERR_FAIL_COND(!collision_object_2);
	if (collision_object_1->interacts_with(collision_object_2)) {
		b2Body *body1 = contact->GetFixtureA()->GetBody();
		b2Body *body2 = contact->GetFixtureB()->GetBody();
		Transform2D transform_a = box2d::collider_get_transform(handle, contact->GetFixtureA()) * collision_object_1->get_transform();
		Transform2D transform_b = box2d::collider_get_transform(handle, contact->GetFixtureB()) * collision_object_2->get_transform();
		Vector2 allowed_local_n1 = transform_a.columns[1].normalized();
		Vector2 allowed_local_n2 = transform_b.columns[1].normalized();
		bool contact_is_pass_through = false;
		b2WorldManifold worldManifold;
		contact->GetWorldManifold(&worldManifold);
		real_t dist = MIN(worldManifold.separations[0], worldManifold.separations[1]);

		ERR_FAIL_COND(collision_object_1->is_shape_disabled(shape1));
		ERR_FAIL_COND(collision_object_2->is_shape_disabled(shape2));
		real_t one_way_margin_1 = collision_object_1->get_shape_one_way_collision_margin(shape1);
		real_t one_way_margin_2 = collision_object_2->get_shape_one_way_collision_margin(shape2);
		if (collision_object_1->is_shape_set_as_one_way_collision(shape1)) {
			Vector2 linvel = box2d::b2Vec2_to_Vector2(body2->GetLinearVelocity());
			real_t motion_len = linvel.length();
			real_t max_allowed = motion_len * MAX(linvel.normalized().dot(allowed_local_n1), 0.0) + one_way_margin_1;
			contact_is_pass_through = linvel.normalized().dot(allowed_local_n1) <= CMP_EPSILON * 10.0 || dist < -max_allowed;
		}
		if (collision_object_2->is_shape_set_as_one_way_collision(shape2) && !contact_is_pass_through) {
			Vector2 linvel = box2d::b2Vec2_to_Vector2(body1->GetLinearVelocity());
			real_t motion_len = linvel.length();
			real_t max_allowed = motion_len * MAX(linvel.normalized().dot(allowed_local_n2), 0.0) + one_way_margin_2;
			contact_is_pass_through = linvel.normalized().dot(allowed_local_n2) <= CMP_EPSILON * 10.0 || dist < -max_allowed;
		}
		if (contact_is_pass_through) {
			contact->SetEnabled(false);
			return;
		}

		if (collision_object_1->get_type() == Box2DCollisionObject2D::TYPE_BODY && collision_object_2->get_type() == Box2DCollisionObject2D::TYPE_BODY) {
			Box2DBody2D *body1 = static_cast<Box2DBody2D *>(collision_object_1);
			Box2DBody2D *body2 = static_cast<Box2DBody2D *>(collision_object_2);
			if (body1->get_static_linear_velocity() != Vector2()) {
				body2->to_add_static_constant_linear_velocity(body1->get_static_linear_velocity());
			}
			if (body2->get_static_linear_velocity() != Vector2()) {
				body1->to_add_static_constant_linear_velocity(body2->get_static_linear_velocity());
			}
			if (body1->get_static_angular_velocity() != 0.0) {
				body2->to_add_static_constant_angular_velocity(body1->get_static_angular_velocity());
			}
			if (body2->get_static_angular_velocity() != 0.0) {
				body1->to_add_static_constant_angular_velocity(body2->get_static_angular_velocity());
			}
		}
	}
}
void Box2DSpace2D::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
	box2d::ContactForceEventInfo force_info = force_info_from_contact(contact);
	if (!force_info.is_valid) {
		return;
	}
	bool send_contacts = contact_force_event_callback(handle, &force_info);
	if (send_contacts) {
		box2d::ContactPointInfo point_info;
		b2WorldManifold worldManifold;
		contact->GetWorldManifold(&worldManifold);
		for (int i = 0; i < contact->GetManifold()->pointCount; i++) {
			point_info.distance_1 = worldManifold.separations[i] * 0.5;
			point_info.distance_2 = worldManifold.separations[i] * 0.5;
			point_info.normal_1 = -worldManifold.normal;
			point_info.normal_2 = worldManifold.normal;
			point_info.impulse_1 = impulse->normalImpulses[i];
			point_info.impulse_2 = impulse->normalImpulses[i];
			point_info.tangent_impulse_1 = impulse->tangentImpulses[i];
			point_info.tangent_impulse_2 = impulse->tangentImpulses[i];
			point_info.local_pos_1 = worldManifold.points[i] - point_info.distance_1 * point_info.normal_1;
			point_info.local_pos_2 = worldManifold.points[i] - point_info.distance_2 * point_info.normal_2;
			point_info.velocity_pos_1 = contact->GetFixtureA()->GetBody()->GetLinearVelocityFromLocalPoint(point_info.local_pos_1);
			point_info.velocity_pos_2 = contact->GetFixtureB()->GetBody()->GetLinearVelocityFromLocalPoint(point_info.local_pos_2);
			contact_point_callback(handle, &point_info, &force_info);
		}
	}
}

void Box2DSpace2D::collision_event_callback(b2World *world_handle, const box2d::CollisionEventInfo *event_info) {
	Box2DSpace2D *space = Box2DPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND(!space);

	uint32_t shape1 = 0;
	Box2DCollisionObject2D *pObject1 = nullptr;
	if (box2d::is_user_data_valid(event_info->user_data1)) {
		pObject1 = Box2DCollisionObject2D::get_collider_user_data(event_info->user_data1, shape1);
	}

	uint32_t shape2 = 0;
	Box2DCollisionObject2D *pObject2 = nullptr;
	if (box2d::is_user_data_valid(event_info->user_data2)) {
		pObject2 = Box2DCollisionObject2D::get_collider_user_data(event_info->user_data2, shape2);
	}

	b2Fixture *collider_handle1 = event_info->collider1;
	b2Fixture *collider_handle2 = event_info->collider2;

	RID rid1, rid2;
	ObjectID instanceId1;
	ObjectID instanceId2;
	Box2DCollisionObject2D::Type type1, type2;

	if (event_info->is_removed) {
		// Try to get backup info for missing objects
		if (!pObject1) {
			space->get_removed_collider_info(collider_handle1, rid1, instanceId1, shape1, type1);
		} else {
			rid1 = pObject1->get_rid();
			instanceId1 = pObject1->get_instance_id();
			type1 = pObject1->get_type();
		}
		if (!pObject2) {
			space->get_removed_collider_info(collider_handle2, rid2, instanceId2, shape2, type2);
		} else {
			rid2 = pObject2->get_rid();
			instanceId2 = pObject2->get_instance_id();
			type2 = pObject2->get_type();
		}
	} else {
		ERR_FAIL_COND(!pObject1);
		rid1 = pObject1->get_rid();
		instanceId1 = pObject1->get_instance_id();
		type1 = pObject1->get_type();

		ERR_FAIL_COND(!pObject2);
		rid2 = pObject2->get_rid();
		instanceId2 = pObject2->get_instance_id();
		type2 = pObject2->get_type();
	}

	if (event_info->is_sensor) {
		if (!instanceId1.is_valid()) {
			ERR_FAIL_COND_MSG(pObject2, "Should be able to get info about a removed object if the other one is still valid.");
			return;
		}
		if (!instanceId2.is_valid()) {
			ERR_FAIL_COND_MSG(pObject2, "Should be able to get info about a removed object if the other one is still valid.");
			return;
		}

		if (type1 != Box2DCollisionObject2D::TYPE_AREA) {
			ERR_FAIL_COND(type2 != Box2DCollisionObject2D::TYPE_AREA);
			SWAP(pObject1, pObject2);
			SWAP(type1, type2);
			SWAP(shape1, shape2);
			SWAP(collider_handle1, collider_handle2);
			SWAP(rid1, rid2);
			SWAP(instanceId1, instanceId2);
		}

		Box2DArea2D *pArea = static_cast<Box2DArea2D *>(pObject1);
		if (type2 == Box2DCollisionObject2D::TYPE_AREA) {
			Box2DArea2D *pArea2 = static_cast<Box2DArea2D *>(pObject2);
			if (event_info->is_started) {
				ERR_FAIL_COND(!pArea);
				ERR_FAIL_COND(!pArea2);
				pArea->on_area_enter(collider_handle2, pArea2, shape2, rid2, instanceId2, collider_handle1, shape1);
				pArea2->on_area_enter(collider_handle1, pArea, shape1, rid1, instanceId1, collider_handle2, shape2);
			} else {
				if (pArea) {
					pArea->on_area_exit(collider_handle2, pArea2, shape2, rid2, instanceId2, collider_handle1, shape1);
				} else {
					// Try to retrieve area if not destroyed yet
					pArea = space->get_area_from_rid(rid1);
					if (pArea) {
						// Use invalid area case to keep counters consistent for already removed collider
						pArea->on_area_exit(collider_handle2, nullptr, shape2, rid2, instanceId2, collider_handle1, shape1);
					}
				}
				if (pArea2) {
					pArea2->on_area_exit(collider_handle1, pArea, shape1, rid1, instanceId1, collider_handle2, shape2);
				} else {
					// Try to retrieve area if not destroyed yet
					pArea2 = space->get_area_from_rid(rid2);
					if (pArea2) {
						// Use invalid area case to keep counters consistent for already removed collider
						pArea2->on_area_exit(collider_handle1, nullptr, shape1, rid1, instanceId1, collider_handle2, shape2);
					}
				}
			}
		} else {
			Box2DBody2D *pBody = static_cast<Box2DBody2D *>(pObject2);
			if (event_info->is_started) {
				ERR_FAIL_COND(!pArea);
				pArea->on_body_enter(collider_handle2, pBody, shape2, rid2, instanceId2, collider_handle1, shape1);
			} else if (pArea) {
				pArea->on_body_exit(collider_handle2, pBody, shape2, rid2, instanceId2, collider_handle1, shape1);
			} else {
				// Try to retrieve area if not destroyed yet
				pArea = space->get_area_from_rid(rid1);
				if (pArea) {
					// Use invalid body case to keep counters consistent for already removed collider
					pArea->on_body_exit(collider_handle2, nullptr, shape2, rid2, instanceId2, collider_handle1, shape1, false);
				}
			}
		}
	} else {
		// Body contacts use contact_force_event_callback instead
		ERR_FAIL_MSG("Shouldn't receive rigidbody collision events.");
	}
}

bool Box2DSpace2D::contact_force_event_callback(b2World *world_handle, const box2d::ContactForceEventInfo *event_info) {
	Box2DSpace2D *space = Box2DPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND_V(!space, false);

	bool send_contacts = false;

#ifdef DEBUG_ENABLED
	if (space->is_debugging_contacts()) {
		send_contacts = true;
	}
#endif

	uint32_t shape1 = 0;
	Box2DCollisionObject2D *pObject1 = nullptr;
	if (box2d::is_user_data_valid(event_info->user_data1)) {
		pObject1 = Box2DCollisionObject2D::get_collider_user_data(event_info->user_data1, shape1);
	}
	ERR_FAIL_COND_V(!pObject1, false);
	ERR_FAIL_COND_V(pObject1->get_type() != Box2DCollisionObject2D::TYPE_BODY, false);

	uint32_t shape2 = 0;
	Box2DCollisionObject2D *pObject2 = nullptr;
	if (box2d::is_user_data_valid(event_info->user_data2)) {
		pObject2 = Box2DCollisionObject2D::get_collider_user_data(event_info->user_data2, shape2);
	}
	ERR_FAIL_COND_V(!pObject2, false);
	ERR_FAIL_COND_V(pObject2->get_type() != Box2DCollisionObject2D::TYPE_BODY, false);

	if (static_cast<Box2DBody2D *>(pObject1)->can_report_contacts()) {
		send_contacts = true;
	}

	if (static_cast<Box2DBody2D *>(pObject2)->can_report_contacts()) {
		send_contacts = true;
	}

	return send_contacts;
}

bool Box2DSpace2D::contact_point_callback(b2World *world_handle, const box2d::ContactPointInfo *contact_info, const box2d::ContactForceEventInfo *event_info) {
	Box2DSpace2D *space = Box2DPhysicsServer2D::singleton->get_active_space(world_handle);
	ERR_FAIL_COND_V(!space, false);

	Vector2 pos1(contact_info->local_pos_1.x, contact_info->local_pos_1.y);
	Vector2 pos2(contact_info->local_pos_2.x, contact_info->local_pos_2.y);

	bool keep_sending_contacts = false;

#ifdef DEBUG_ENABLED
	if (space->is_debugging_contacts()) {
		keep_sending_contacts = true;
		space->add_debug_contact(pos1);
		space->add_debug_contact(pos2);
	}
#endif
	// body and shape 1
	uint32_t shape1 = 0;
	Box2DCollisionObject2D *pObject1 = nullptr;
	if (box2d::is_user_data_valid(event_info->user_data1)) {
		pObject1 = Box2DCollisionObject2D::get_collider_user_data(event_info->user_data1, shape1);
	}
	ERR_FAIL_COND_V(!pObject1, false);
	ERR_FAIL_COND_V(pObject1->get_type() != Box2DCollisionObject2D::TYPE_BODY, false);
	Box2DBody2D *body1 = static_cast<Box2DBody2D *>(pObject1);
	// body and shape 2
	uint32_t shape2 = 0;
	Box2DCollisionObject2D *pObject2 = nullptr;
	if (box2d::is_user_data_valid(event_info->user_data2)) {
		pObject2 = Box2DCollisionObject2D::get_collider_user_data(event_info->user_data2, shape2);
	}
	ERR_FAIL_COND_V(!pObject2, false);
	ERR_FAIL_COND_V(pObject2->get_type() != Box2DCollisionObject2D::TYPE_BODY, false);
	Box2DBody2D *body2 = static_cast<Box2DBody2D *>(pObject2);

	real_t depth_1 = MAX(0.0, -contact_info->distance_1); // negative distance means penetration
	real_t depth_2 = MAX(0.0, -contact_info->distance_2); // negative distance means penetration

	Vector2 normal_1(contact_info->normal_1.x, contact_info->normal_1.y);
	Vector2 normal_2(contact_info->normal_2.x, contact_info->normal_2.y);
	Vector2 tangent_1 = normal_1.orthogonal();
	Vector2 tangent_2 = normal_2.orthogonal();
	Vector2 impulse_1 = contact_info->impulse_1 * normal_1 + contact_info->tangent_impulse_1 * tangent_1;
	Vector2 impulse_2 = contact_info->impulse_2 * normal_2 + contact_info->tangent_impulse_2 * tangent_2;

	if (body1->can_report_contacts()) {
		keep_sending_contacts = true;
		Vector2 vel_pos2(contact_info->velocity_pos_2.x, contact_info->velocity_pos_2.y);
		body1->add_contact(pos1, normal_1, depth_1, (int)shape1, pos2, (int)shape2, body2->get_instance_id(), body2->get_rid(), vel_pos2, impulse_1);
	}

	if (body2->can_report_contacts()) {
		keep_sending_contacts = true;
		Vector2 vel_pos1(contact_info->velocity_pos_1.x, contact_info->velocity_pos_1.y);
		body2->add_contact(pos2, normal_2, depth_2, (int)shape2, pos1, (int)shape1, body1->get_instance_id(), body1->get_rid(), vel_pos1, impulse_2);
	}

	return keep_sending_contacts;
}

void Box2DSpace2D::step(real_t p_step) {
	last_step = p_step;
	for (SelfList<Box2DBody2D> *body_iterator = active_list.first(); body_iterator;) {
		Box2DBody2D *body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->reset_contact_count();
	}
	contact_debug_count = 0;

	ProjectSettings *project_settings = ProjectSettings::get_singleton();

	default_gravity_dir = project_settings->get_setting_with_override("physics/2d/default_gravity_vector");
	default_gravity_value = project_settings->get_setting_with_override("physics/2d/default_gravity");

	default_linear_damping = project_settings->get_setting_with_override("physics/2d/default_linear_damp");
	default_angular_damping = project_settings->get_setting_with_override("physics/2d/default_angular_damp");

	for (SelfList<Box2DBody2D> *body_iterator = mass_properties_update_list.first(); body_iterator;) {
		Box2DBody2D *body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->update_mass_properties();
	}

	for (SelfList<Box2DArea2D> *area_iterator = area_update_list.first(); area_iterator;) {
		Box2DArea2D *area = area_iterator->self();
		area_iterator = area_iterator->next();
		area->update_area_override();
	}

	for (SelfList<Box2DBody2D> *body_iterator = body_area_update_list.first(); body_iterator;) {
		Box2DBody2D *body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->update_area_override();
	}

	for (SelfList<Box2DBody2D> *body_iterator = gravity_update_list.first(); body_iterator;) {
		Box2DBody2D *body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->update_gravity(p_step);
	}

	box2d::SimulationSettings settings;
	settings.dt = p_step;
	settings.max_position_iterations = Box2DProjectSettings::get_position_iterations();
	settings.max_velocity_iterations = Box2DProjectSettings::get_velocity_iterations();
	settings.gravity.x = default_gravity_dir.x * default_gravity_value;
	settings.gravity.y = default_gravity_dir.y * default_gravity_value;

	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
	box2d::world_step(handle, &settings);

	// Needed only for one physics step to retrieve lost info
	removed_colliders.clear();

	for (SelfList<Box2DBody2D> *body_iterator = active_list.first(); body_iterator;) {
		Box2DBody2D *body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->on_update_active();
	}
	active_objects = box2d::world_get_active_objects_count(handle);
}

// Returns true to ignore the collider
bool Box2DSpace2D::_is_handle_excluded_callback(b2World *world_handle, b2Fixture *collider_handle, b2FixtureUserData user_data, const box2d::QueryExcludedInfo *handle_excluded_info) {
	for (uint32_t exclude_index = 0; exclude_index < handle_excluded_info->query_exclude_size; ++exclude_index) {
		if (box2d::are_handles_equal(handle_excluded_info->query_exclude[exclude_index], collider_handle)) {
			return true;
		}
	}

	ERR_FAIL_COND_V(!box2d::is_user_data_valid(user_data), false);

	uint32_t shape_index = 0;
	Box2DCollisionObject2D *collision_object_2d = Box2DCollisionObject2D::get_collider_user_data(user_data, shape_index);
	ERR_FAIL_COND_V(!collision_object_2d, false);

	if (handle_excluded_info->query_canvas_instance_id != ((uint64_t)collision_object_2d->get_canvas_instance_id())) {
		return true;
	}

	if (0 == (collision_object_2d->get_collision_layer() & handle_excluded_info->query_collision_layer_mask)) {
		return true;
	}

	if (handle_excluded_info->query_exclude_body == collision_object_2d->get_rid().get_id()) {
		return true;
	}

	return Box2DPhysicsServer2D::singleton->get_active_space(world_handle)->get_direct_state()->is_body_excluded_from_query(collision_object_2d->get_rid());
}

void Box2DSpace2D::call_queries() {
	for (SelfList<Box2DBody2D> *body_iterator = state_query_list.first(); body_iterator;) {
		Box2DBody2D *body = body_iterator->self();
		body_iterator = body_iterator->next();
		body->call_queries();
	}

	for (SelfList<Box2DArea2D> *area_iterator = monitor_query_list.first(); area_iterator;) {
		Box2DArea2D *area = area_iterator->self();
		area_iterator = area_iterator->next();
		area->call_queries();
	}
}

void Box2DSpace2D::set_param(PhysicsServer2D::SpaceParameter p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer2D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
			contact_recycle_radius = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
			contact_max_separation = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
			contact_max_allowed_penetration = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
			contact_bias = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			body_linear_velocity_sleep_threshold = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			body_angular_velocity_sleep_threshold = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			body_time_to_sleep = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
			constraint_bias = p_value;
			break;
		case PhysicsServer2D::SPACE_PARAM_SOLVER_ITERATIONS:
			solver_iterations = p_value;
			break;
	}
}

real_t Box2DSpace2D::get_param(PhysicsServer2D::SpaceParameter p_param) const {
	switch (p_param) {
		case PhysicsServer2D::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
			return contact_recycle_radius;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_SEPARATION:
			return contact_max_separation;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION:
			return contact_max_allowed_penetration;
		case PhysicsServer2D::SPACE_PARAM_CONTACT_DEFAULT_BIAS:
			return contact_bias;
		case PhysicsServer2D::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
			return body_linear_velocity_sleep_threshold;
		case PhysicsServer2D::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
			return body_angular_velocity_sleep_threshold;
		case PhysicsServer2D::SPACE_PARAM_BODY_TIME_TO_SLEEP:
			return body_time_to_sleep;
		case PhysicsServer2D::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
			return constraint_bias;
		case PhysicsServer2D::SPACE_PARAM_SOLVER_ITERATIONS:
			return solver_iterations;
	}
	return 0;
}

void Box2DSpace2D::set_default_area_param(PhysicsServer2D::AreaParameter p_param, const Variant &p_value) {
	switch (p_param) {
		case PhysicsServer2D::AREA_PARAM_GRAVITY: {
			default_gravity_value = p_value;
		} break;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_VECTOR: {
			default_gravity_dir = p_value;
		} break;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP: {
			default_linear_damping = p_value;
		} break;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP: {
			default_angular_damping = p_value;
		} break;
		default:
			ERR_FAIL_MSG("Unsupported space default area param " + itos(p_param));
	}
}

Variant Box2DSpace2D::get_default_area_param(PhysicsServer2D::AreaParameter p_param) const {
	switch (p_param) {
		case PhysicsServer2D::AREA_PARAM_GRAVITY:
			return default_gravity_value;
		case PhysicsServer2D::AREA_PARAM_GRAVITY_VECTOR:
			return default_gravity_dir;
		case PhysicsServer2D::AREA_PARAM_LINEAR_DAMP:
			return default_linear_damping;
		case PhysicsServer2D::AREA_PARAM_ANGULAR_DAMP:
			return default_angular_damping;
		default:
			break;
	}

	ERR_FAIL_V_MSG(Variant(), "Unsupported space default area param " + itos(p_param));
}

Box2DArea2D *Box2DSpace2D::get_area_from_rid(RID p_area_rid) const {
	return Box2DPhysicsServer2D::singleton->area_owner.get_or_null(p_area_rid);
}

Box2DBody2D *Box2DSpace2D::get_body_from_rid(RID p_body_rid) const {
	return Box2DPhysicsServer2D::singleton->body_owner.get_or_null(p_body_rid);
}

Box2DShape2D *Box2DSpace2D::get_shape_from_rid(RID p_shape_rid) const {
	return Box2DPhysicsServer2D::singleton->shape_owner.get_or_null(p_shape_rid);
}

void Box2DSpace2D::lock() {
	locked = true;
}

void Box2DSpace2D::unlock() {
	locked = false;
}

bool Box2DSpace2D::is_locked() const {
	return locked;
}

Box2DDirectSpaceState2D *Box2DSpace2D::get_direct_state() {
	return direct_access;
}

Box2DSpace2D::Box2DSpace2D() {
	ProjectSettings *project_settings = ProjectSettings::get_singleton();

	// Use the default physics step for force application on the first frame
	int physics_fps = project_settings->get_setting_with_override("physics/common/physics_ticks_per_second");
	last_step = physics_fps != 0 ? 1.0 / ((real_t)physics_fps) : 0.001f;

	body_linear_velocity_sleep_threshold = project_settings->get_setting_with_override("physics/2d/sleep_threshold_linear");
	body_angular_velocity_sleep_threshold = project_settings->get_setting_with_override("physics/2d/sleep_threshold_angular");
	body_time_to_sleep = project_settings->get_setting_with_override("physics/2d/time_before_sleep");
	solver_iterations = project_settings->get_setting_with_override("physics/2d/solver/solver_iterations");
	contact_recycle_radius = project_settings->get_setting_with_override("physics/2d/solver/contact_recycle_radius");
	contact_max_separation = project_settings->get_setting_with_override("physics/2d/solver/contact_max_separation");
	contact_max_allowed_penetration = project_settings->get_setting_with_override("physics/2d/solver/contact_max_allowed_penetration");
	contact_bias = project_settings->get_setting_with_override("physics/2d/solver/default_contact_bias");
	constraint_bias = project_settings->get_setting_with_override("physics/2d/solver/default_constraint_bias");

	direct_access = memnew(Box2DDirectSpaceState2D);
	direct_access->space = this;

	ERR_FAIL_COND(box2d::is_handle_valid(handle));

	box2d::WorldSettings world_settings = box2d::default_world_settings();
	world_settings.sleep_linear_threshold = body_linear_velocity_sleep_threshold;
	world_settings.sleep_angular_threshold = body_angular_velocity_sleep_threshold;
	world_settings.sleep_time_until_sleep = body_time_to_sleep;

	handle = box2d::world_create(&world_settings);
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));

	box2d::world_set_active_body_callback(handle, active_body_callback);
	box2d::world_set_collision_filter_callback(handle, this);
	box2d::world_set_contact_listener(handle, this);
}

Box2DSpace2D::~Box2DSpace2D() {
	ERR_FAIL_COND(!box2d::is_handle_valid(handle));
	box2d::world_destroy(handle);
	handle = box2d::invalid_world_handle();

	memdelete(direct_access);
}

bool Box2DSpace2D::test_body_motion(Box2DBody2D *p_body, const Transform2D &p_from, const Vector2 &p_motion, double p_margin, bool p_collide_separation_ray, bool p_recovery_as_collision, PhysicsServer2DExtensionMotionResult *r_result) const {
	if (r_result) {
		r_result->travel = Vector2();
	}
	Transform2D body_transform = p_from; // Because body_transform needs to be modified during recovery
	// Step 1: recover motion.
	// Expand the body colliders by the margin (grow) and check if now it collides with a collider,
	// if yes, "recover" / "push" out of this collider
	Vector2 recover_motion;
	real_t margin = MAX(p_margin, TEST_MOTION_MARGIN_MIN_VALUE);

	bool recovered = Box2DBodyUtils2D::body_motion_recover(*this, *p_body, body_transform, p_motion, margin, recover_motion);
	// Step 2: Cast motion.
	// Try to to find what is the possible motion (how far it can move, it's a shapecast, when you try to find the safe point (max you can move without collision ))
	real_t best_safe = 1.0;
	real_t best_unsafe = 1.0;
	int best_body_shape = -1;
	Box2DBodyUtils2D::cast_motion(*this, *p_body, body_transform, p_motion, p_collide_separation_ray, contact_max_allowed_penetration, margin, best_safe, best_unsafe, best_body_shape);

	// Step 3: Rest Info
	// Apply the motion and fill the collision information
	bool collided = false;
	if ((p_recovery_as_collision && recovered) || (best_safe < 1.0)) {
		if (best_safe >= 1.0) {
			best_body_shape = -1; //no best shape with cast, reset to -1
		}

		// Get the rest info in unsafe advance
		Vector2 unsafe_motion = p_motion * best_unsafe;
		body_transform.columns[2] += unsafe_motion;

		collided = Box2DBodyUtils2D::body_motion_collide(*this, *p_body, body_transform, p_motion, best_body_shape, margin, r_result);
	}

	if (r_result) {
		if (collided) {
			r_result->travel += recover_motion + p_motion * best_safe;
			r_result->remainder = p_motion - p_motion * best_safe;
			r_result->collision_safe_fraction = best_safe;
			r_result->collision_unsafe_fraction = best_unsafe;
		} else {
			r_result->travel += recover_motion + p_motion;
			r_result->remainder = Vector2();
			r_result->collision_depth = 0.0f;
			r_result->collision_safe_fraction = 1.0f;
			r_result->collision_unsafe_fraction = 1.0f;
		}
	}

	return collided;
}

int Box2DSpace2D::box2d_intersect_aabb(Rect2 p_aabb, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, box2d::PointHitInfo *p_results, int32_t p_max_results, int32_t *p_result_count, RID p_exclude_body) const {
	ERR_FAIL_COND_V(p_max_results < 1, false);

	b2Vec2 rect_begin{ p_aabb.position.x, p_aabb.position.y };
	b2Vec2 rect_end{ p_aabb.get_end().x, p_aabb.get_end().y };
	box2d::QueryExcludedInfo handle_excluded_info = box2d::default_query_excluded_info();
	handle_excluded_info.query_exclude = (b2Fixture **)memalloc((p_max_results) * sizeof(b2Fixture *));
	handle_excluded_info.query_collision_layer_mask = p_collision_mask;
	handle_excluded_info.query_exclude_size = 0;
	handle_excluded_info.query_exclude_body = p_exclude_body.get_id();

	return box2d::intersect_aabb(handle, rect_begin, rect_end, p_collide_with_bodies, p_collide_with_areas, p_results, p_max_results, Box2DSpace2D::_is_handle_excluded_callback, &handle_excluded_info);
}
