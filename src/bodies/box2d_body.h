#pragma once

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "../joints/box2d_joint.h"
#include "../spaces/box2d_space.h"
#include "box2d_collision_object.h"

using namespace godot;

class Box2DDirectBodyState;

class Box2DBody : public Box2DCollisionObject {
	PhysicsServer2D::BodyMode mode = PhysicsServer2D::BODY_MODE_RIGID;

	SelfList<Box2DBody> active_list;
	SelfList<Box2DBody> direct_state_query_list;

	bool active = true;
	bool can_sleep = true;
	PhysicsServer2D::CCDMode collision_mode = PhysicsServer2D::CCD_MODE_DISABLED;

	Transform2D new_transform;

	Callable body_state_callback;

	Box2DDirectBodyState *direct_state = nullptr;
	HashSet<Box2DJoint *> joints;
	int32 max_contacts_reported = 0;

public:
	// Physics Server
	void set_max_contacts_reported(int32 p_max_contacts_reported);

	int32 get_max_contacts_reported();

	void set_space(Box2DSpace *p_space) override;

	void set_state_sync_callback(const Callable &p_callable);

	Box2DDirectBodyState *get_direct_state();

	void set_active(bool p_active);
	bool is_active() const;

	void wakeup();

	void set_mode(PhysicsServer2D::BodyMode p_mode);
	PhysicsServer2D::BodyMode get_mode() const;

	void set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer2D::BodyState p_state) const;

	void set_continuous_collision_detection_mode(PhysicsServer2D::CCDMode mode);
	PhysicsServer2D::CCDMode get_continuous_collision_detection_mode() const;

	void add_joint(Box2DJoint *p_joint);
	void remove_joint(Box2DJoint *p_joint);

	virtual HashSet<Box2DJoint *> get_joints() override;

	void after_step();
	void call_queries();

	Box2DBody();
	~Box2DBody();
};
