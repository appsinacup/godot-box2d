#ifndef BOX2D_BODY_H
#define BOX2D_BODY_H

#include <godot_cpp/classes/physics_server2d.hpp>

#include "box2d_collision_object.h"
#include "box2d_space.h"

using namespace godot;

class Box2DDirectBodyState;

class Box2DBody: public Box2DCollisionObject {
	Transform2D new_transform;

	Box2DDirectBodyState *direct_state = nullptr;
	friend class Box2DDirectBodyState; // i give up, too many functions to expose
public:
	Box2DDirectBodyState *get_direct_state();

	void set_state(PhysicsServer2D::BodyState p_state, const Variant &p_variant);
	Variant get_state(PhysicsServer2D::BodyState p_state) const;

	void set_space(Box2DSpace* p_space) override;

	Box2DBody();
	~Box2DBody();
};

#endif // BOX2D_BODY_H
