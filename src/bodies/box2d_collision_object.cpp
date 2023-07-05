#include "box2d_collision_object.h"

#include "../b2_user_settings.h"

#include "../box2d_type_conversions.h"
#include "../spaces/box2d_direct_space_state.h"
#include "box2d_area.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_contact.h>
#include <box2d/b2_fixture.h>

// Mass

void Box2DCollisionObject::reset_mass_properties() {
	mass_data.mass = 1;
	mass_data.I = 0;
	mass_data.center = b2Vec2();
	if (body) {
		body->SetMassData(&mass_data);
	}
}

void Box2DCollisionObject::set_mass(real_t p_mass) {
	if (mass_data.mass == p_mass) {
		return;
	}
	mass_data.mass = p_mass;
	if (body) {
		body->SetMassData(&mass_data);
	}
}
double Box2DCollisionObject::get_mass() const {
	return mass_data.mass; // no need to convert
}
void Box2DCollisionObject::set_inertia(real_t p_inertia) {
	if (mass_data.I == p_inertia) {
		return;
	}
	mass_data.I = p_inertia;
	if (body) {
		body->SetMassData(&mass_data);
	}
}
double Box2DCollisionObject::get_inertia() const {
	return mass_data.I; // no need to convert
}
void Box2DCollisionObject::set_center_of_mass(Vector2 p_center_of_mass) {
	if (godot_to_box2d(p_center_of_mass) == mass_data.center) {
		return;
	}
	godot_to_box2d(p_center_of_mass, mass_data.center);
	if (body) {
		body->SetMassData(&mass_data);
	}
}
Vector2 Box2DCollisionObject::get_center_of_mass() const {
	if (body) {
		return box2d_to_godot(mass_data.center + body->GetPosition());
	} else {
		return box2d_to_godot(mass_data.center + body_def->position);
	}
}

// Damping

void Box2DCollisionObject::set_linear_damp_mode(PhysicsServer2D::BodyDampMode p_linear_damp) {
	if (damping.linear_damp_mode == p_linear_damp) {
		return;
	}
	damping.linear_damp_mode = p_linear_damp;
	recalculate_total_linear_damp();
}
void Box2DCollisionObject::set_angular_damp_mode(PhysicsServer2D::BodyDampMode p_angular_damp) {
	if (damping.angular_damp_mode == p_angular_damp) {
		return;
	}
	damping.angular_damp_mode = p_angular_damp;
	recalculate_total_angular_damp();
}

void Box2DCollisionObject::set_linear_damp(real_t p_linear_damp) {
	if (damping.linear_damp == p_linear_damp) {
		return;
	}
	damping.linear_damp = p_linear_damp;
	recalculate_total_linear_damp();
}
void Box2DCollisionObject::set_angular_damp(real_t p_angular_damp) {
	if (damping.angular_damp == p_angular_damp) {
		return;
	}
	damping.angular_damp = p_angular_damp;
	recalculate_total_angular_damp();
}
PhysicsServer2D::BodyDampMode Box2DCollisionObject::get_linear_damp_mode() const {
	return damping.linear_damp_mode;
}
PhysicsServer2D::BodyDampMode Box2DCollisionObject::get_angular_damp_mode() const {
	return damping.angular_damp_mode;
}
double Box2DCollisionObject::get_linear_damp() const {
	return damping.linear_damp;
}
double Box2DCollisionObject::get_angular_damp() const {
	return damping.angular_damp;
}

void Box2DCollisionObject::recalculate_total_linear_damp() {
	total_linear_damp = damping.linear_damp;
	if (get_linear_damp_mode() == PhysicsServer2D::BodyDampMode::BODY_DAMP_MODE_REPLACE) {
		body_def->linearDamping = total_linear_damp;
		if (body) {
			body->SetLinearDamping(body_def->linearDamping);
		}
		// replace linear damp with body one
		return;
	}
	bool keep_computing = true;
	for (Box2DArea *area : areas) {
		if (!keep_computing) {
			break;
		}
		real_t linear_damp = area->damping.linear_damp;
		switch (area->get_linear_damp_override_mode()) {
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE: {
				total_linear_damp += linear_damp;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
				total_linear_damp += linear_damp;
				keep_computing = false;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_REPLACE: {
				total_linear_damp = linear_damp;
				keep_computing = false;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
				total_linear_damp = linear_damp;
			} break;
			default: {
			}
		}
	}
	body_def->linearDamping = total_linear_damp;
	if (body) {
		body->SetLinearDamping(body_def->linearDamping);
	}
}

void Box2DCollisionObject::recalculate_total_angular_damp() {
	total_angular_damp = damping.angular_damp;
	if (get_angular_damp_mode() == PhysicsServer2D::BodyDampMode::BODY_DAMP_MODE_REPLACE) {
		body_def->angularDamping = total_angular_damp;
		if (body) {
			body->SetAngularDamping(body_def->angularDamping);
		}
		// replace angular damp with body one
		return;
	}
	// compute angular damp from areas
	bool keep_computing = true;
	for (Box2DArea *area : areas) {
		if (!keep_computing) {
			break;
		}
		real_t angular_damp = area->damping.angular_damp;
		switch (area->get_angular_damp_override_mode()) {
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE: {
				total_angular_damp += angular_damp;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
				total_angular_damp += angular_damp;
				keep_computing = false;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_REPLACE: {
				total_angular_damp = angular_damp;
				keep_computing = false;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
				total_angular_damp = angular_damp;
			} break;
			default: {
			}
		}
	}
	body_def->angularDamping = total_angular_damp;
	if (body) {
		body->SetAngularDamping(body_def->angularDamping);
	}
}

// Collision

void Box2DCollisionObject::set_priority(real_t p_priority) {
	collision.priority = p_priority;
}

double Box2DCollisionObject::get_priority() const {
	return collision.priority;
}

// Physics Material

void Box2DCollisionObject::set_bounce(real_t p_bounce) {
	if (physics_material.bounce == p_bounce) {
		return;
	}
	physics_material.bounce = p_bounce;
	if (body) {
		_update_shapes();
	}
}
void Box2DCollisionObject::set_friction(real_t p_friction) {
	if (physics_material.friction == p_friction) {
		return;
	}
	physics_material.friction = p_friction;
	if (body) {
		_update_shapes();
	}
}

double Box2DCollisionObject::get_bounce() const {
	return physics_material.bounce;
}
double Box2DCollisionObject::get_friction() const {
	return physics_material.friction;
}

// Direct Body API

Vector2 Box2DCollisionObject::get_total_gravity() const {
	return Vector2(total_gravity.x, total_gravity.y);
}

double Box2DCollisionObject::get_total_linear_damp() const {
	return box2d_to_godot(body_def->linearDamping);
}

double Box2DCollisionObject::get_total_angular_damp() const {
	return box2d_to_godot(body_def->linearDamping);
}

Vector2 Box2DCollisionObject::get_center_of_mass_local() const {
	return box2d_to_godot(mass_data.center);
}

double Box2DCollisionObject::get_inverse_mass() const {
	if (mass_data.mass <= 0) {
		return 0;
	}
	return 1.0 / mass_data.mass;
}
double Box2DCollisionObject::get_inverse_inertia() const {
	if (mass_data.I <= 0) {
		return 0;
	}
	return 1.0 / mass_data.I;
}
void Box2DCollisionObject::set_linear_velocity(const Vector2 &p_linear_velocity) {
	b2Vec2 box2d_linear_velocity = godot_to_box2d(p_linear_velocity);
	body_def->linearVelocity = box2d_linear_velocity;
	if (body) {
		body->SetLinearVelocity(box2d_linear_velocity);
	}
}

Vector2 Box2DCollisionObject::get_linear_velocity() const {
	if (body) {
		return box2d_to_godot(body->GetLinearVelocity());
	}
	return Vector2();
}
void Box2DCollisionObject::set_angular_velocity(double p_velocity) {
	float angularVelocity = godot_to_box2d(p_velocity);
	body_def->angularVelocity = angularVelocity;
	if (body) {
		body->SetAngularVelocity(angularVelocity);
	}
}
double Box2DCollisionObject::get_angular_velocity() const {
	if (body) {
		return box2d_to_godot(body->GetAngularVelocity());
	}
	return 0;
}
void Box2DCollisionObject::set_transform(const Transform2D &transform) {
	_set_transform(transform);
}
Transform2D Box2DCollisionObject::get_transform() const {
	if (body) {
		return Transform2D(body->GetAngle(), box2d_to_godot(body->GetPosition()));
	} else {
		return Transform2D(body_def->angle, box2d_to_godot(body_def->position));
	}
}
Vector2 Box2DCollisionObject::get_velocity_at_local_position(const Vector2 &p_local_position) const {
	if (body) {
		b2Vec2 velocity = body->GetLinearVelocityFromLocalPoint(godot_to_box2d(p_local_position));
		return box2d_to_godot(velocity);
	}
	return Vector2();
}
void Box2DCollisionObject::apply_central_impulse(const Vector2 &impulse) {
	if (body) {
		body->ApplyLinearImpulseToCenter(body->GetMass() * godot_to_box2d(impulse), true);
	}
}
void Box2DCollisionObject::apply_impulse(const Vector2 &impulse, const Vector2 &position) {
	if (body) {
		body->ApplyLinearImpulse(body->GetMass() * godot_to_box2d(impulse), godot_to_box2d(position), true);
	}
}
void Box2DCollisionObject::apply_torque_impulse(double impulse) {
	if (body) {
		body->ApplyTorque(body->GetMass() * godot_to_box2d(impulse), true);
	}
}
void Box2DCollisionObject::apply_central_force(const Vector2 &force) {
	if (body) {
		body->ApplyForceToCenter(body->GetMass() * godot_to_box2d(force), true);
	}
}
void Box2DCollisionObject::apply_force(const Vector2 &force, const Vector2 &position) {
	if (body) {
		body->ApplyForce(body->GetMass() * godot_to_box2d(force), godot_to_box2d(position), true);
	}
}
void Box2DCollisionObject::apply_torque(double torque) {
	if (body) {
		body->ApplyTorque(body->GetMass() * godot_to_box2d(torque), true);
	}
}

// Constant Forces

void Box2DCollisionObject::add_constant_central_force(const Vector2 &force) {
	constant_forces.constant_force += godot_to_box2d(force);
	// TODO set position to center
	//constant_force_position = position;
}
void Box2DCollisionObject::add_constant_force(const Vector2 &force, const Vector2 &position) {
	constant_forces.constant_force += godot_to_box2d(force);
	constant_forces.constant_force_position = godot_to_box2d(position);
}
void Box2DCollisionObject::add_constant_torque(double torque) {
	constant_forces.constant_torque += godot_to_box2d(torque);
}
void Box2DCollisionObject::set_constant_force(const Vector2 &force) {
	constant_forces.constant_force = godot_to_box2d(force);
}
Vector2 Box2DCollisionObject::get_constant_force() const {
	return box2d_to_godot(constant_forces.constant_force);
}
void Box2DCollisionObject::set_constant_torque(double torque) {
	constant_forces.constant_torque = godot_to_box2d(torque);
}
double Box2DCollisionObject::get_constant_torque() const {
	return box2d_to_godot(constant_forces.constant_torque);
}
void Box2DCollisionObject::set_sleep_state(bool enabled) {
	if (body) {
		body->SetAwake(!enabled);
	}
}
bool Box2DCollisionObject::is_sleeping() const {
	return !body->IsAwake();
}
Box2DCollisionObject::ContactEdgeData Box2DCollisionObject::_get_contact_edge_data(int32_t contact_idx) const {
	if (!body) {
		return ContactEdgeData();
	}
	b2ContactEdge *contacts = body->GetContactList();
	int32 contacts_count = 0;
	b2WorldManifold worldManifold;
	while (contacts) {
		contacts_count += contacts->contact->GetManifold()->pointCount;
		if (contacts_count > contact_idx) {
			return ContactEdgeData{ contacts, contacts_count - contact_idx - 1 };
		}
		contacts = contacts->next;
	}
	return ContactEdgeData();
}

int32_t Box2DCollisionObject::get_contact_count() const {
	if (!body) {
		return 0;
	}
	b2ContactEdge *contacts = body->GetContactList();
	int32 contacts_count = 0;
	while (contacts) {
		contacts_count += contacts->contact->GetManifold()->pointCount;
		contacts = contacts->next;
	}
	return contacts_count;
}
Vector2 Box2DCollisionObject::get_contact_local_position(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return Vector2();
	}
	b2WorldManifold worldManifold;
	data.edge->contact->GetWorldManifold(&worldManifold);
	return box2d_to_godot(worldManifold.points[data.point_idx]);
}
Vector2 Box2DCollisionObject::get_contact_local_normal(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return Vector2();
	}
	b2Vec2 normal = data.edge->contact->GetManifold()->localNormal;
	return Vector2(normal.x, normal.y);
}
int32_t Box2DCollisionObject::get_contact_local_shape(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return -1;
	}
	return data.edge->contact->GetFixtureA()->GetUserData().shape_idx;
}
RID Box2DCollisionObject::get_contact_collider(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return RID();
	}
	b2BodyUserData *user_data = (b2BodyUserData *)&data.edge->other->GetUserData();
	return user_data->collision_object->get_self();
}
Vector2 Box2DCollisionObject::get_contact_collider_position(int32_t contact_idx) const {
	return get_contact_local_position(contact_idx);
}
uint64_t Box2DCollisionObject::get_contact_collider_id(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return 0;
	}
	b2BodyUserData *user_data = (b2BodyUserData *)&data.edge->other->GetUserData();
	return user_data->collision_object->get_object_instance_id();
}
Object *Box2DCollisionObject::get_contact_collider_object(int32_t contact_idx) const {
	ObjectID id = ObjectID(get_contact_collider_id(contact_idx));
	return ObjectDB::get_instance(id);
}
int32_t Box2DCollisionObject::get_contact_collider_shape(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return -1;
	}
	return data.edge->contact->GetFixtureB()->GetUserData().shape_idx;
}
Vector2 Box2DCollisionObject::get_contact_collider_velocity_at_position(int32_t contact_idx) const {
	ContactEdgeData data = _get_contact_edge_data(contact_idx);
	if (!data.edge) {
		return Vector2();
	}
	b2WorldManifold worldManifold;
	data.edge->contact->GetWorldManifold(&worldManifold);
	b2Vec2 world_point = worldManifold.points[data.point_idx];
	return box2d_to_godot(data.edge->other->GetLinearVelocityFromWorldPoint(world_point));
}
Vector2 Box2DCollisionObject::get_contact_impulse(int32_t contact_idx) const {
	return get_contact_local_normal(contact_idx) * get_step();
}
double Box2DCollisionObject::get_step() const {
	Box2DSpace *space = get_space();
	if (!space) {
		return 0;
	}
	return space->get_step();
}
void Box2DCollisionObject::integrate_forces() {
}

PhysicsDirectSpaceState2D *Box2DCollisionObject::get_space_state() {
	if (!direct_space) {
		direct_space = memnew(Box2DDirectSpaceState);
		direct_space->space = space;
	}
	return direct_space;
}

// Physics Server

void Box2DCollisionObject::set_collision_layer(uint32_t layer) {
	filter.categoryBits = layer;
	if (body) {
		_update_shapes();
	}
}

uint32_t Box2DCollisionObject::get_collision_layer() const {
	return filter.categoryBits;
}
void Box2DCollisionObject::set_collision_mask(uint32_t layer) {
	filter.maskBits = layer;
	if (body) {
		_update_shapes();
	}
}

uint32_t Box2DCollisionObject::get_collision_mask() const {
	return filter.maskBits;
}

void Box2DCollisionObject::set_pickable(bool p_pickable) {
	collision.pickable = p_pickable;
	if (body) {
		_update_shapes();
	}
}

void Box2DCollisionObject::set_object_instance_id(const ObjectID &p_instance_id) {
	object_instance_id = p_instance_id;
}
ObjectID Box2DCollisionObject::get_object_instance_id() const { return object_instance_id; }

Object *Box2DCollisionObject::get_object() const {
	ObjectID id = ObjectID(object_instance_id);
	return ObjectDB::get_instance(id);
}
Object *Box2DCollisionObject::get_object_unsafe() const {
	return reinterpret_cast<Object *>(internal::gde_interface->object_get_instance_from_id(object_instance_id));
	;
}

void Box2DCollisionObject::set_canvas_instance_id(const ObjectID &p_instance_id) {
	canvas_instance_id = p_instance_id;
}
ObjectID Box2DCollisionObject::get_canvas_instance_id() const { return canvas_instance_id; }

Box2DSpace *Box2DCollisionObject::get_space() const { return space; }

void Box2DCollisionObject::add_shape(Box2DShape *p_shape, const Transform2D &p_transform, bool p_disabled) {
	Shape s;
	s.shape = p_shape;
	s.xform = p_transform;
	s.disabled = p_disabled;
	shapes.push_back(s);

	// TODO (queue) update
}

void Box2DCollisionObject::set_shape(int p_index, Box2DShape *p_shape) {
	ERR_FAIL_INDEX(p_index, shapes.size());
	//shapes[p_index].shape->remove_owner(this);
	shapes.write[p_index].shape = p_shape;

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_transform(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes.write[p_index].xform = p_transform;

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_disabled(int p_index, bool p_disabled) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	Shape &shape = shapes.write[p_index];
	if (shape.disabled == p_disabled) {
		return;
	}

	shape.disabled = p_disabled;

	if (!space) {
		return;
	}

	for (int j = 0; j < shape.fixtures.size(); j++) {
		if (body) {
			body->DestroyFixture(shape.fixtures[j]);
		}
		shape.fixtures.write[j] = nullptr;
	}
	shape.fixtures.clear();

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_as_one_way_collision(int p_index, bool enable) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	Shape &shape = shapes.write[p_index];
	if (shape.one_way_collision == enable) {
		return;
	}

	shape.one_way_collision = enable;

	_update_shapes();
}

void Box2DCollisionObject::remove_shape(Box2DShape *p_shape) {
	//remove a shape, all the times it appears
	for (int i = 0; i < shapes.size(); i++) {
		if (shapes[i].shape == p_shape) {
			remove_shape(i);
			i--;
		}
	}
}

void Box2DCollisionObject::recreate_shapes() {
	// todo make it work with one shape
	_clear_fixtures();
	_update_shapes();
}

void Box2DCollisionObject::remove_shape(int p_index) {
	//remove anything from shape to be erased to end, so subindices don't change
	ERR_FAIL_INDEX(p_index, shapes.size());
	for (int i = p_index; i < shapes.size(); i++) {
		Shape &shape = shapes.write[i];
		for (int j = 0; j < shape.fixtures.size(); j++) {
			// should never get here with a null owner
			if (body) {
				body->DestroyFixture(shape.fixtures[j]);
			}
			shape.fixtures.write[j] = nullptr;
		}
		shape.fixtures.clear();
	}
	shapes.remove_at(p_index);

	// TODO: (queue) update
}

void Box2DCollisionObject::_clear_fixtures() {
	for (int i = 0; i < shapes.size(); i++) {
		Shape &shape = shapes.write[i];
		for (int j = 0; j < shape.fixtures.size(); j++) {
			if (body) {
				body->DestroyFixture(shape.fixtures[j]);
			}
			shape.fixtures.write[j] = nullptr;
		}
		shape.fixtures.clear();
	}
}

void Box2DCollisionObject::_set_space(Box2DSpace *p_space) {
	if (space) {
		// NOTE: Remember the transform by copying it from the b2Body to the b2BodyDef.
		if (body) {
			body_def->position = body->GetPosition();
			body_def->angle = body->GetAngle();
		}

		_clear_fixtures();
		space->remove_object(this);
	}
	space = p_space;
	if (space) {
		space->add_object(this);
		_update_shapes();
	}
}

int Box2DCollisionObject::get_shape_count() const { return shapes.size(); }
Box2DShape *Box2DCollisionObject::get_shape(int p_index) const {
	CRASH_BAD_INDEX(p_index, shapes.size());
	return shapes[p_index].shape;
}

const Transform2D &Box2DCollisionObject::get_shape_transform(int p_index) const {
	CRASH_BAD_INDEX(p_index, shapes.size());
	return shapes[p_index].xform;
}

void Box2DCollisionObject::set_gravity_scale(real_t p_gravity_scale) {
	gravity_scale = p_gravity_scale; // no need to convert
}

real_t Box2DCollisionObject::get_gravity_scale() {
	return gravity_scale; // no need to convert
}

void Box2DCollisionObject::add_collision_exception(Box2DCollisionObject *excepted_body) {
	collision_exception.insert(excepted_body);
}
void Box2DCollisionObject::remove_collision_exception(Box2DCollisionObject *excepted_body) {
	collision_exception.erase(excepted_body);
}

bool Box2DCollisionObject::is_body_collision_excepted(Box2DCollisionObject *excepted_body) {
	return collision_exception.has(excepted_body);
}
TypedArray<RID> Box2DCollisionObject::get_collision_exception() {
	TypedArray<RID> array;
	for (Box2DCollisionObject *E : collision_exception) {
		array.append(E->get_self());
	}
	return array;
}
// MISC

void Box2DCollisionObject::_update_shapes() {
	if (!space || !body) {
		return;
	}

	for (int i = 0; i < shapes.size(); i++) {
		Shape &s = shapes.write[i];
		if (s.disabled) {
			continue;
		}

		//not quite correct, should compute the next matrix..
		//Transform2D xform = transform * s.xform;
		bool is_static = body->GetType() == b2_staticBody;
		if (s.fixtures.is_empty()) {
			int box2d_shape_count = s.shape->get_b2Shape_count(is_static);
			s.fixtures.resize(box2d_shape_count);
			for (int j = 0; j < box2d_shape_count; j++) {
				b2FixtureDef fixture_def;
				fixture_def.shape = s.shape->get_transformed_b2Shape(j, s.xform, s.one_way_collision, is_static);
				if (fixture_def.shape == nullptr) {
					ERR_PRINT("Shape " + itos(j) + " disabled.");
					s.disabled = true;
					break;
				}
				fixture_def.density = 1.0f;
				fixture_def.filter = filter;
				fixture_def.friction = physics_material.friction;
				fixture_def.restitution = physics_material.bounce;
				fixture_def.isSensor = type == Type::TYPE_AREA;
				fixture_def.userData.shape_idx = i;
				fixture_def.userData.box2d_fixture_idx = j;
				s.fixtures.write[j] = body->CreateFixture(&fixture_def);
			}
		} else {
			int box2d_shape_count = s.shape->get_b2Shape_count(is_static);
			for (int j = 0; j < box2d_shape_count; j++) {
				b2Fixture *fixture = s.fixtures[j];
				fixture->SetFilterData(filter);
				fixture->SetFriction(physics_material.friction);
				fixture->SetRestitution(physics_material.bounce);
			}
		}

		//space->get_broadphase()->move(s.bpid, shape_aabb);
	}
}
void Box2DCollisionObject::before_step() {
	if (body) {
		// custom gravity
		body->ApplyForceToCenter(body->GetMass() * gravity_scale * total_gravity, false);
		if (constant_forces.constant_force != b2Vec2_zero) {
			// constant force
			body->ApplyForce(body->GetMass() * constant_forces.constant_force, constant_forces.constant_force_position + body->GetPosition(), true);
		}
		if (constant_forces.constant_torque != 0) {
			// constant torque
			body->ApplyTorque(body->GetMass() * constant_forces.constant_torque, true);
		}
	}
}

Box2DCollisionObject::Type Box2DCollisionObject::get_type() const { return type; }

void Box2DCollisionObject::set_self(const RID &p_self) { self = p_self; }
RID Box2DCollisionObject::get_self() const { return self; }

void Box2DCollisionObject::add_area(Box2DArea *p_area) {
	areas.append(p_area);
	areas.sort();
	recalculate_total_gravity();
	recalculate_total_angular_damp();
	recalculate_total_linear_damp();
}
void Box2DCollisionObject::sort_areas() {
	areas.sort();
}
void Box2DCollisionObject::recalculate_total_gravity() {
	total_gravity = b2Vec2_zero;
	// compute gravity from other areas
	bool keep_computing = true;
	for (Box2DArea *area : areas) {
		if (!keep_computing) {
			break;
		}
		b2Vec2 area_gravity = area->get_b2_gravity(get_transform());

		switch (area->get_gravity_override_mode()) {
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE: {
				total_gravity += area_gravity;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_COMBINE_REPLACE: {
				total_gravity += area_gravity;
				keep_computing = false;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_REPLACE: {
				total_gravity = area_gravity;
				keep_computing = false;
			} break;
			case PhysicsServer2D::AreaSpaceOverrideMode::AREA_SPACE_OVERRIDE_REPLACE_COMBINE: {
				total_gravity = area_gravity;
			} break;
			default: {
			}
		}
	}
}
void Box2DCollisionObject::remove_area(Box2DArea *p_area) {
	areas.erase(p_area);
	areas.sort();
	recalculate_total_gravity();
	recalculate_total_angular_damp();
	recalculate_total_linear_damp();
}

b2BodyDef *Box2DCollisionObject::get_b2BodyDef() { return body_def; }
void Box2DCollisionObject::set_b2BodyDef(b2BodyDef *p_body_def) { body_def = p_body_def; }
b2Body *Box2DCollisionObject::get_b2Body() { return body; }
void Box2DCollisionObject::set_b2Body(b2Body *p_body) {
	body = p_body;
	// set additional properties here
	if (body) {
		body->SetMassData(&mass_data);
		body->SetAwake(true);
		//recreate_shapes();
	}
}

void Box2DCollisionObject::_set_transform(const Transform2D &p_transform, bool p_update_shapes) {
	if (body) {
		Vector2 pos = p_transform.get_origin();
		b2Vec2 box2d_pos;
		godot_to_box2d(pos, box2d_pos);
		body->SetTransform(box2d_pos, p_transform.get_rotation());
	} else {
		godot_to_box2d(p_transform.get_origin(), body_def->position);
		body_def->angle = p_transform.get_rotation();
	}
	if (p_update_shapes) {
		_update_shapes();
	}
}

Box2DCollisionObject::Box2DCollisionObject(Type p_type) {
	type = p_type;
	body_def = memnew(b2BodyDef);
	body_def->userData.collision_object = this;
	reset_mass_properties();
}

Box2DCollisionObject::~Box2DCollisionObject() {
	for (Box2DArea *area : areas) {
		if (area) {
			area->remove_body(this);
		}
	}
	memdelete(body_def);
}
