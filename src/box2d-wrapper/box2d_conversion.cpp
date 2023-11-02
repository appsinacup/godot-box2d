#include "box2d_wrapper.h"
#include "box2d_conversion.h"
#include <box2d.h>

using namespace box2d;

Handle box2d::world_handle_to_handle(b2WorldId handle) {
    return Handle {
        handle.index,
        0,
        0,
        handle.revision,
    };
}

b2WorldId box2d::handle_to_world_handle(Handle handle) {
    return b2WorldId {
        handle.world_index,
        handle.revision,
    };
}

Handle box2d::body_handle_to_handle(b2BodyId handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2BodyId box2d::handle_to_body_handle(Handle handle) {
    return b2BodyId {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

Handle box2d::joint_handle_to_handle(b2JointId handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2JointId box2d::handle_to_joint_handle(Handle handle) {
    return b2JointId {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

Handle box2d::shape_handle_to_handle(b2ShapeId handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2ShapeId box2d::handle_to_shape_handle(Handle handle) {
    return b2ShapeId {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

b2BodyType box2d::body_type_to_b2_body_type(BodyType body_type) {
    switch (body_type) {
        case BodyType::Dynamic: return b2BodyType::b2_dynamicBody;
        case BodyType::Static: return b2BodyType::b2_staticBody;
        case BodyType::Kinematic: return b2BodyType::b2_kinematicBody;
    }
    return b2BodyType::b2_bodyTypeCount;
}

b2Vec2 box2d::vector_to_b2_vec(Vector vector) {
    return b2Vec2{vector.x, vector.y};
}

Vector box2d::b2_vec_to_vector(b2Vec2 vector) {
    return Vector{vector.x, vector.y};
}
