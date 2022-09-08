#ifndef BOX2D_BODY_H
#define BOX2D_BODY_H

#include "box2d_collision_object.h"
#include "box2d_space.h"

using namespace godot;

class Box2DBody: public Box2DCollisionObject {
public:
    void set_space(Box2DSpace* p_space) override;

    Box2DBody();
    ~Box2DBody();
};

#endif // BOX2D_BODY_H
