#include <box2d.h>


namespace box2d {
    class Handle;
    enum class BodyType;
    class Vector;

    Handle world_handle_to_handle(b2WorldId handle);

    b2WorldId handle_to_world_handle(Handle handle);

    Handle body_handle_to_handle(b2BodyId handle);

    b2BodyId handle_to_body_handle(Handle handle);

    Handle joint_handle_to_handle(b2JointId handle);

    b2JointId handle_to_joint_handle(Handle handle);

    Handle shape_handle_to_handle(b2ShapeId handle);

    b2ShapeId handle_to_shape_handle(Handle handle);

    b2BodyType body_type_to_b2_body_type(BodyType body_type);

    b2Vec2 vector_to_b2_vec(Vector vector);

    Vector b2_vec_to_vector(b2Vec2 vector);
}
