#pragma once

#include <godot_cpp/classes/physics_direct_body_state2d_extension.hpp>
#include <godot_cpp/classes/physics_direct_space_state2d.hpp>

#include "box2d_body.h"

using namespace godot;

class Box2DDirectBodyState : public PhysicsDirectBodyState2DExtension {
	GDCLASS(Box2DDirectBodyState, PhysicsDirectBodyState2DExtension);

protected:
	static void _bind_methods() {}

public:
	Box2DBody *body = nullptr;

	virtual Vector2 _get_total_gravity() const override;
	virtual double _get_total_linear_damp() const override;
	virtual double _get_total_angular_damp() const override;
	virtual Vector2 _get_center_of_mass() const override;
	virtual Vector2 _get_center_of_mass_local() const override;
	virtual double _get_inverse_mass() const override;
	virtual double _get_inverse_inertia() const override;
	virtual void _set_linear_velocity(const Vector2 &velocity) override;
	virtual Vector2 _get_linear_velocity() const override;
	virtual void _set_angular_velocity(double velocity) override;
	virtual double _get_angular_velocity() const override;
	virtual void _set_transform(const Transform2D &transform) override;
	virtual Transform2D _get_transform() const override;
	virtual Vector2 _get_velocity_at_local_position(const Vector2 &local_position) const override;
	virtual void _apply_central_impulse(const Vector2 &impulse) override;
	virtual void _apply_impulse(const Vector2 &impulse, const Vector2 &position) override;
	virtual void _apply_torque_impulse(double impulse) override;
	virtual void _apply_central_force(const Vector2 &force) override;
	virtual void _apply_force(const Vector2 &force, const Vector2 &position) override;
	virtual void _apply_torque(double torque) override;
	virtual void _add_constant_central_force(const Vector2 &force) override;
	virtual void _add_constant_force(const Vector2 &force, const Vector2 &position) override;
	virtual void _add_constant_torque(double torque) override;
	virtual void _set_constant_force(const Vector2 &force) override;
	virtual Vector2 _get_constant_force() const override;
	virtual void _set_constant_torque(double torque) override;
	virtual double _get_constant_torque() const override;
	virtual void _set_sleep_state(bool enabled) override;
	virtual bool _is_sleeping() const override;
	virtual int32_t _get_contact_count() const override;
	virtual Vector2 _get_contact_local_position(int32_t contact_idx) const override;
	virtual Vector2 _get_contact_local_normal(int32_t contact_idx) const override;
	virtual int32_t _get_contact_local_shape(int32_t contact_idx) const override;
	virtual RID _get_contact_collider(int32_t contact_idx) const override;
	virtual Vector2 _get_contact_collider_position(int32_t contact_idx) const override;
	virtual uint64_t _get_contact_collider_id(int32_t contact_idx) const override;
	virtual Object *_get_contact_collider_object(int32_t contact_idx) const override;
	virtual int32_t _get_contact_collider_shape(int32_t contact_idx) const override;
	virtual Vector2 _get_contact_collider_velocity_at_position(int32_t contact_idx) const override;
	virtual Vector2 _get_contact_impulse(int32_t contact_idx) const override;
	virtual double _get_step() const override;
	virtual void _integrate_forces() override;
	virtual PhysicsDirectSpaceState2D *_get_space_state() override;

	~Box2DDirectBodyState() override = default;
};
