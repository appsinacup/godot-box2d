#pragma once

#include <box2d/b2_world_callbacks.h>

class Box2DSpaceContactFilter : public b2ContactFilter {
public:
	/// Return true if contact calculations should be performed between these two shapes.
	/// @warning for performance reasons this is only called when the AABBs begin to overlap.
	virtual bool ShouldCollide(b2Fixture *fixtureA, b2Fixture *fixtureB) override;
};
