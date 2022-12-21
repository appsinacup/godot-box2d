#ifndef PHYSICS_SERVER_BOX2D_H
#define PHYSICS_SERVER_BOX2D_H

// We don't need windows.h in this plugin but many others do and it throws up on itself all the time
// So best to include it and make sure CI warns us when we use something Microsoft took for their own goals....
#ifdef WIN32
#include <windows.h>
#endif

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension.hpp>
#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include <godot_cpp/variant/callable.hpp>

#include <godot_cpp/core/binder_common.hpp>

#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/rid_owner.hpp>

#include "box2d_shape.h"
#include "box2d_space.h"
#include "box2d_body.h"

using namespace godot;

class PhysicsServerBox2D : public PhysicsServer2DExtension
{
	GDCLASS(PhysicsServerBox2D, PhysicsServer2DExtension);

	bool active = true;
	bool doing_sync = false;

	bool using_threads = false;

	bool flushing_queries = false;

	HashSet<const Box2DSpace *> active_spaces;

	mutable RID_PtrOwner<Box2DShape, true> shape_owner;
	mutable RID_PtrOwner<Box2DSpace, true> space_owner;
	mutable RID_PtrOwner<Box2DBody, true> body_owner;

	RID _shape_create(ShapeType p_shape);

protected:
	static void _bind_methods() {};

public:
	virtual RID _circle_shape_create() override;
	virtual RID _rectangle_shape_create() override;
	virtual void _shape_set_data(const RID &shape, const Variant &data) override;
	virtual Variant _shape_get_data(const RID &shape) const override;

	virtual RID _space_create() override;
	virtual void _space_set_active(const RID &p_space, bool p_active) override;
	virtual bool _space_is_active(const RID &p_space) const override;

	virtual RID _body_create() override;
	virtual void _body_set_space(const RID &body, const RID &space) override;
	virtual RID _body_get_space(const RID &body) const override;

	virtual void _body_set_mode(const RID &p_body, BodyMode p_mode) override;
	virtual BodyMode _body_get_mode(const RID &p_body) const override;

	virtual void _body_add_shape(const RID &body, const RID &shape, const Transform2D &transform, bool disabled) override;
	virtual void _body_set_shape(const RID &body, int64_t shape_idx, const RID &shape) override;
	virtual void _body_set_shape_transform(const RID &body, int64_t shape_idx, const Transform2D &transform) override;
	virtual int64_t _body_get_shape_count(const RID &body) const override;
	virtual RID _body_get_shape(const RID &body, int64_t shape_idx) const override;
	virtual Transform2D _body_get_shape_transform(const RID &body, int64_t shape_idx) const override;
	virtual void _body_remove_shape(const RID &body, int64_t shape_idx) override;
	virtual void _body_clear_shapes(const RID &body) override;

	virtual void _body_set_state(const RID &p_body, PhysicsServer2D::BodyState p_state, const Variant &p_value) override;
	virtual Variant _body_get_state(const RID &p_body, PhysicsServer2D::BodyState p_state) const override;

	virtual void _body_set_state_sync_callback(const RID &p_body, const Callable &p_callable) override;

	virtual PhysicsDirectBodyState2D *_body_get_direct_state(const RID &p_body) override;

	virtual void _free_rid(const RID &p_rid) override;

	virtual void _set_active(bool p_active) override;
	virtual void _init() override;
	virtual void _step(double p_step) override;
	virtual void _sync() override;
	virtual void _flush_queries() override;
	virtual void _end_sync() override;
	virtual void _finish() override;

	virtual bool _is_flushing_queries() const override { return flushing_queries; };

	PhysicsServerBox2D();
	~PhysicsServerBox2D();
};

class PhysicsServerBox2DFactory: public Object {
	GDCLASS(PhysicsServerBox2DFactory, Object);

protected:
	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("create_box2d_callback"), &PhysicsServerBox2DFactory::_create_box2d_callback);
	}

public:
	PhysicsServer2D *_create_box2d_callback() {
		PhysicsServer2D *physics_server_2d = memnew(PhysicsServerBox2D());
		return physics_server_2d;
	}

};

#endif // ! PHYSICS_SERVER_BOX2D_H
