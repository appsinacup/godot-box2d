#ifndef BOX2D_BODY_H
#define BOX2D_BODY_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/variant.hpp>

#include "box2d_collision_object.h"
#include "box2d_space.h"

using namespace godot;

class Box2DDirectBodyState;

class Box2DBody: public Box2DCollisionObject {
	PhysicsServer2D::BodyMode mode = PhysicsServer2D::BODY_MODE_RIGID;

	SelfList<Box2DBody> active_list;
	SelfList<Box2DBody> direct_state_query_list;

	bool active = true;
	bool can_sleep = true;

	Transform2D new_transform;

	Callable body_state_callback;

	Box2DDirectBodyState *direct_state = nullptr;
	friend class Box2DDirectBodyState; // i give up, too many functions to expose
public:
	void set_state_sync_callback(const Callable &p_callable);

	Box2DDirectBodyState *get_direct_state();

	void set_linear_velocity(const Vector2 &p_linear_velocity);
	Vector2 get_linear_velocity() const;

	void set_angular_velocity(real_t p_angular_velocity);
	real_t get_angular_velocity() const;

	void set_active(bool p_active);
	_FORCE_INLINE_ bool is_active() const { return active; }

	_FORCE_INLINE_ void wakeup() {
		if ((!get_space()) || mode == PhysicsServer2D::BODY_MODE_STATIC || mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
			return;
		}
		set_active(true);
	}

	void set_mode(PhysicsServer2D::BodyMode p_mode);
	PhysicsServer2D::BodyMode get_mode() const;

	void set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer2D::BodyState p_state) const;

	void set_space(Box2DSpace* p_space) override;

	void after_step();
	void call_queries();

	Box2DBody();
	~Box2DBody();
};

#endif // BOX2D_BODY_H
