#ifndef BOX2D_SPACE_H
#define BOX2D_SPACE_H

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/variant/rid.hpp>

#include <box2d/b2_world.h>

using namespace godot;

class Box2DSpace {
private:
    RID self;

    b2World *world = nullptr;
public:
    _FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
    _FORCE_INLINE_ RID get_self() const { return self; }

    Box2DSpace();
    ~Box2DSpace();
};

#endif // BOX2D_SPACE_H
