#include "box2d_wrapper.h"
#include "box2d_holder.h"
#include <box2d.h>

using namespace box2d;

Handle Box2DHolder::world_to_handle(b2World* world) {
    return Handle {
        handle.index,
        0,
        0,
        handle.revision,
    };
}

b2World* Box2DHolder::handle_to_world(Handle handle) {
    return b2WorldId {
        handle.world_index,
        handle.revision,
    };
}

Handle Box2DHolder::body_to_handle(b2Body* handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2Body* Box2DHolder::handle_to_body(Handle handle) {
    return b2BodyId {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

Handle Box2DHolder::joint_to_handle(b2Joint* handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2Joint* Box2DHolder::handle_to_joint(Handle handle) {
    return b2JointId {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

Handle Box2DHolder::shape_to_handle(b2Shape* handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2Shape* Box2DHolder::handle_to_shape(Handle handle) {
    return b2Shape* {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

Handle Box2DHolder::collider_to_handle(b2Fixture* handle) {
    return Handle {
        0,
        handle.index,
        handle.world,
        handle.revision,
    };
}

b2Fixture* Box2DHolder::handle_to_collider(Handle handle) {
    return b2Fixture* {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

b2Transform* Box2DHolder::handle_to_collider_transform(Handle handle) {
    return b2Fixture* {
        handle.object_index,
        handle.world,
        handle.revision,
    };
}

b2BodyType Box2DHolder::body_type_to_b2_body_type(BodyType body_type) {
    switch (body_type) {
        case BodyType::Dynamic: return b2BodyType::b2_dynamicBody;
        case BodyType::Static: return b2BodyType::b2_staticBody;
        case BodyType::Kinematic: return b2BodyType::b2_kinematicBody;
    }
    return b2BodyType::b2_staticBody;
}

b2Vec2 Box2DHolder::vector_to_b2_vec(Vector vector) {
    return b2Vec2{vector.x, vector.y};
}

Vector Box2DHolder::b2_vec_to_vector(b2Vec2 vector) {
    return Vector{vector.x, vector.y};
}
