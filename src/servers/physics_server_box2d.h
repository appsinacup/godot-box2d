#pragma once

// We don't need windows.h in this plugin but many others do and it throws up on itself all the time
// So best to include it and make sure CI warns us when we use something Microsoft took for their own goals....
#ifdef WIN32
#include <windows.h>
#endif

#include <godot_cpp/classes/physics_direct_body_state2d.hpp>
#include <godot_cpp/classes/physics_direct_space_state2d.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension.hpp>
#include <godot_cpp/classes/physics_server2d_extension_motion_result.hpp>
#include <godot_cpp/variant/callable.hpp>

#include <godot_cpp/core/binder_common.hpp>

#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/rid_owner.hpp>

#include "../bodies/box2d_area.h"
#include "../bodies/box2d_body.h"
#include "../joints/box2d_joint.h"
#include "../shapes/box2d_shape.h"
#include "../spaces/box2d_space.h"

using namespace godot;

class PhysicsServerBox2D : public PhysicsServer2DExtension {
	GDCLASS(PhysicsServerBox2D, PhysicsServer2DExtension);
	bool active = true;
	bool doing_sync = false;

	bool using_threads = false;

	bool flushing_queries = false;

	HashSet<const Box2DSpace *> active_spaces;
	Box2DArea default_area;

	mutable RID_PtrOwner<Box2DShape, true> shape_owner;
	mutable RID_PtrOwner<Box2DSpace, true> space_owner;
	mutable RID_PtrOwner<Box2DArea, true> area_owner;
	mutable RID_PtrOwner<Box2DBody, true> body_owner;
	mutable RID_PtrOwner<Box2DJoint, true> joint_owner;

	RID _shape_create(ShapeType p_shape);

protected:
	static void _bind_methods(){};

public:
	/* SHAPE API */
	virtual RID _world_boundary_shape_create() override;
	virtual RID _separation_ray_shape_create() override;
	virtual RID _segment_shape_create() override;
	virtual RID _circle_shape_create() override;
	virtual RID _rectangle_shape_create() override;
	virtual RID _capsule_shape_create() override;
	virtual RID _convex_polygon_shape_create() override;
	virtual RID _concave_polygon_shape_create() override;
	virtual void _shape_set_data(const RID &shape, const Variant &data) override;
	virtual void _shape_set_custom_solver_bias(const RID &shape, double bias) override;
	virtual PhysicsServer2D::ShapeType _shape_get_type(const RID &shape) const override;
	virtual Variant _shape_get_data(const RID &shape) const override;
	virtual double _shape_get_custom_solver_bias(const RID &shape) const override;
	virtual bool _shape_collide(const RID &shape_A, const Transform2D &xform_A, const Vector2 &motion_A, const RID &shape_B, const Transform2D &xform_B, const Vector2 &motion_B, void *results, int32_t result_max, int32_t *result_count) override;

	/* SPACE API */
	virtual RID _space_create() override;
	virtual void _space_set_active(const RID &space, bool active) override;
	virtual bool _space_is_active(const RID &space) const override;
	virtual void _space_set_param(const RID &space, PhysicsServer2D::SpaceParameter param, double value) override;
	virtual double _space_get_param(const RID &space, PhysicsServer2D::SpaceParameter param) const override;
	virtual PhysicsDirectSpaceState2D *_space_get_direct_state(const RID &space) override;
	virtual void _space_set_debug_contacts(const RID &space, int32_t max_contacts) override;
	virtual PackedVector2Array _space_get_contacts(const RID &space) const override;
	virtual int32_t _space_get_contact_count(const RID &space) const override;

	/* AREA API */
	virtual RID _area_create() override;
	virtual void _area_set_space(const RID &area, const RID &space) override;
	virtual RID _area_get_space(const RID &area) const override;
	virtual void _area_add_shape(const RID &area, const RID &shape, const Transform2D &transform, bool disabled) override;
	virtual void _area_set_shape(const RID &area, int32_t shape_idx, const RID &shape) override;
	virtual void _area_set_shape_transform(const RID &area, int32_t shape_idx, const Transform2D &transform) override;
	virtual void _area_set_shape_disabled(const RID &area, int32_t shape_idx, bool disabled) override;
	virtual int32_t _area_get_shape_count(const RID &area) const override;
	virtual RID _area_get_shape(const RID &area, int32_t shape_idx) const override;
	virtual Transform2D _area_get_shape_transform(const RID &area, int32_t shape_idx) const override;
	virtual void _area_remove_shape(const RID &area, int32_t shape_idx) override;
	virtual void _area_clear_shapes(const RID &area) override;
	virtual void _area_attach_object_instance_id(const RID &area, uint64_t id) override;
	virtual uint64_t _area_get_object_instance_id(const RID &area) const override;
	virtual void _area_attach_canvas_instance_id(const RID &area, uint64_t id) override;
	virtual uint64_t _area_get_canvas_instance_id(const RID &area) const override;
	virtual void _area_set_param(const RID &area, PhysicsServer2D::AreaParameter param, const Variant &value) override;
	virtual void _area_set_transform(const RID &area, const Transform2D &transform) override;
	virtual Variant _area_get_param(const RID &area, PhysicsServer2D::AreaParameter param) const override;
	virtual Transform2D _area_get_transform(const RID &area) const override;
	virtual void _area_set_collision_layer(const RID &area, uint32_t layer) override;
	virtual uint32_t _area_get_collision_layer(const RID &area) const override;
	virtual void _area_set_collision_mask(const RID &area, uint32_t mask) override;
	virtual uint32_t _area_get_collision_mask(const RID &area) const override;
	virtual void _area_set_monitorable(const RID &area, bool monitorable) override;
	virtual void _area_set_pickable(const RID &area, bool pickable) override;
	virtual void _area_set_monitor_callback(const RID &area, const Callable &callback) override;
	virtual void _area_set_area_monitor_callback(const RID &area, const Callable &callback) override;

	/* BODY API */
	virtual RID _body_create() override;
	virtual void _body_set_space(const RID &body, const RID &space) override;
	virtual RID _body_get_space(const RID &body) const override;
	virtual void _body_set_mode(const RID &body, PhysicsServer2D::BodyMode mode) override;
	virtual PhysicsServer2D::BodyMode _body_get_mode(const RID &body) const override;
	virtual void _body_add_shape(const RID &body, const RID &shape, const Transform2D &transform, bool disabled) override;
	virtual void _body_set_shape(const RID &body, int32_t shape_idx, const RID &shape) override;
	virtual void _body_set_shape_transform(const RID &body, int32_t shape_idx, const Transform2D &transform) override;
	virtual int32_t _body_get_shape_count(const RID &body) const override;
	virtual RID _body_get_shape(const RID &body, int32_t shape_idx) const override;
	virtual Transform2D _body_get_shape_transform(const RID &body, int32_t shape_idx) const override;
	virtual void _body_set_shape_disabled(const RID &body, int32_t shape_idx, bool disabled) override;
	virtual void _body_set_shape_as_one_way_collision(const RID &body, int32_t shape_idx, bool enable, double margin) override;
	virtual void _body_remove_shape(const RID &body, int32_t shape_idx) override;
	virtual void _body_clear_shapes(const RID &body) override;
	virtual void _body_attach_object_instance_id(const RID &body, uint64_t id) override;
	virtual uint64_t _body_get_object_instance_id(const RID &body) const override;
	virtual void _body_attach_canvas_instance_id(const RID &body, uint64_t id) override;
	virtual uint64_t _body_get_canvas_instance_id(const RID &body) const override;
	virtual void _body_set_continuous_collision_detection_mode(const RID &body, PhysicsServer2D::CCDMode mode) override;
	virtual PhysicsServer2D::CCDMode _body_get_continuous_collision_detection_mode(const RID &body) const override;
	virtual void _body_set_collision_layer(const RID &body, uint32_t layer) override;
	virtual uint32_t _body_get_collision_layer(const RID &body) const override;
	virtual void _body_set_collision_mask(const RID &body, uint32_t mask) override;
	virtual uint32_t _body_get_collision_mask(const RID &body) const override;
	virtual void _body_set_collision_priority(const RID &body, double priority) override;
	virtual double _body_get_collision_priority(const RID &body) const override;
	virtual void _body_set_param(const RID &body, PhysicsServer2D::BodyParameter param, const Variant &value) override;
	virtual Variant _body_get_param(const RID &body, PhysicsServer2D::BodyParameter param) const override;
	virtual void _body_reset_mass_properties(const RID &body) override;
	virtual void _body_set_state(const RID &body, PhysicsServer2D::BodyState state, const Variant &value) override;
	virtual Variant _body_get_state(const RID &body, PhysicsServer2D::BodyState state) const override;
	virtual void _body_apply_central_impulse(const RID &body, const Vector2 &impulse) override;
	virtual void _body_apply_torque_impulse(const RID &body, double impulse) override;
	virtual void _body_apply_impulse(const RID &body, const Vector2 &impulse, const Vector2 &position) override;
	virtual void _body_apply_central_force(const RID &body, const Vector2 &force) override;
	virtual void _body_apply_force(const RID &body, const Vector2 &force, const Vector2 &position) override;
	virtual void _body_apply_torque(const RID &body, double torque) override;
	virtual void _body_add_constant_central_force(const RID &body, const Vector2 &force) override;
	virtual void _body_add_constant_force(const RID &body, const Vector2 &force, const Vector2 &position) override;
	virtual void _body_add_constant_torque(const RID &body, double torque) override;
	virtual void _body_set_constant_force(const RID &body, const Vector2 &force) override;
	virtual Vector2 _body_get_constant_force(const RID &body) const override;
	virtual void _body_set_constant_torque(const RID &body, double torque) override;
	virtual double _body_get_constant_torque(const RID &body) const override;
	virtual void _body_set_axis_velocity(const RID &body, const Vector2 &axis_velocity) override;
	virtual void _body_add_collision_exception(const RID &body, const RID &excepted_body) override;
	virtual void _body_remove_collision_exception(const RID &body, const RID &excepted_body) override;
	virtual TypedArray<RID> _body_get_collision_exceptions(const RID &body) const override;
	virtual void _body_set_max_contacts_reported(const RID &body, int32_t amount) override;
	virtual int32_t _body_get_max_contacts_reported(const RID &body) const override;
	virtual void _body_set_contacts_reported_depth_threshold(const RID &body, double threshold) override;
	virtual double _body_get_contacts_reported_depth_threshold(const RID &body) const override;
	virtual void _body_set_omit_force_integration(const RID &body, bool enable) override;
	virtual bool _body_is_omitting_force_integration(const RID &body) const override;
	virtual void _body_set_state_sync_callback(const RID &body, const Callable &callable) override;
	virtual void _body_set_force_integration_callback(const RID &body, const Callable &callable, const Variant &userdata) override;
	virtual bool _body_collide_shape(const RID &body, int32_t body_shape, const RID &shape, const Transform2D &shape_xform, const Vector2 &motion, void *results, int32_t result_max, int32_t *result_count) override;
	virtual void _body_set_pickable(const RID &body, bool pickable) override;
	virtual PhysicsDirectBodyState2D *_body_get_direct_state(const RID &body) override;
	virtual bool _body_test_motion(const RID &body, const Transform2D &from, const Vector2 &motion, double margin, bool collide_separation_ray, bool recovery_as_collision, PhysicsServer2DExtensionMotionResult *result) const override;

	/* JOINT API */
	virtual RID _joint_create() override;
	virtual void _joint_clear(const RID &joint) override;
	virtual void _joint_set_param(const RID &joint, PhysicsServer2D::JointParam param, double value) override;
	virtual double _joint_get_param(const RID &joint, PhysicsServer2D::JointParam param) const override;
	virtual void _joint_disable_collisions_between_bodies(const RID &joint, bool disable) override;
	virtual bool _joint_is_disabled_collisions_between_bodies(const RID &joint) const override;
	virtual void _joint_make_pin(const RID &joint, const Vector2 &anchor, const RID &body_a, const RID &body_b) override;
	virtual void _joint_make_groove(const RID &joint, const Vector2 &a_groove1, const Vector2 &a_groove2, const Vector2 &b_anchor, const RID &body_a, const RID &body_b) override;
	virtual void _joint_make_damped_spring(const RID &joint, const Vector2 &anchor_a, const Vector2 &anchor_b, const RID &body_a, const RID &body_b) override;
	virtual void _pin_joint_set_param(const RID &joint, PhysicsServer2D::PinJointParam param, double value) override;
	virtual double _pin_joint_get_param(const RID &joint, PhysicsServer2D::PinJointParam param) const override;
	virtual void _damped_spring_joint_set_param(const RID &joint, PhysicsServer2D::DampedSpringParam param, double value) override;
	virtual double _damped_spring_joint_get_param(const RID &joint, PhysicsServer2D::DampedSpringParam param) const override;
	virtual PhysicsServer2D::JointType _joint_get_type(const RID &joint) const override;

	/* MISC API */
	virtual void _free_rid(const RID &rid) override;
	virtual void _set_active(bool active) override;
	virtual void _init() override;
	virtual void _step(double step) override;
	virtual void _sync() override;
	virtual void _flush_queries() override;
	virtual void _end_sync() override;
	virtual void _finish() override;
	virtual bool _is_flushing_queries() const override;
	virtual int32_t _get_process_info(PhysicsServer2D::ProcessInfo process_info) override;

	PhysicsServerBox2D();
	~PhysicsServerBox2D();
};

class PhysicsServerBox2DFactory : public Object {
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
