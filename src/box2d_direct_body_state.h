#ifndef BOX2D_DIRECT_BODY_STATE_H
#define BOX2D_DIRECT_BODY_STATE_H

#include <godot_cpp/classes/physics_direct_body_state2d_extension.hpp>

#include "box2d_body.h"

using namespace godot;

class Box2DDirectBodyState : public PhysicsDirectBodyState2DExtension {
	GDCLASS(Box2DDirectBodyState, PhysicsDirectBodyState2DExtension);

protected:
	static void _bind_methods() {}

public:
	Box2DBody *body = nullptr;

	virtual void _set_linear_velocity(const Vector2 &p_velocity) override;
	virtual Vector2 _get_linear_velocity() const override;

	virtual void _set_angular_velocity(double p_velocity) override; // should be real_t
	virtual double _get_angular_velocity() const override; // should be real_t

	virtual void _set_transform(const Transform2D &p_transform) override;
	virtual Transform2D _get_transform() const override;

	virtual void _set_sleep_state(bool p_enable) override;
	virtual bool _is_sleeping() const override;

	~Box2DDirectBodyState() override = default;
};

#endif // BOX2D_DIRECT_BODY_STATE_H
