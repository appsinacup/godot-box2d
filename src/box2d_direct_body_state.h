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

	virtual void _set_transform(const Transform2D &p_transform) override;
	virtual Transform2D _get_transform() const override;

	~Box2DDirectBodyState() override = default;
};

#endif // BOX2D_DIRECT_BODY_STATE_H
