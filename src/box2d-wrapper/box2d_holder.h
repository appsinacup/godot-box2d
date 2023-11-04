#include <box2d.h>

namespace box2d {
class Handle;
enum class BodyType;
class Vector;

class Box2DHolder {
public:
	Handle world_to_handle(b2World *handle);
	b2World *handle_to_world(Handle handle);

	Handle body_to_handle(b2Body *handle);
	b2Body *handle_to_body(Handle handle);

	Handle joint_to_handle(b2Joint *handle);
	b2Joint *handle_to_joint(Handle handle);

	Handle shape_to_handle(b2Shape *handle);
	b2Shape *handle_to_shape(Handle handle);

	Handle collider_to_handle(b2Fixture *handle);
	b2Fixture *handle_to_collider(Handle handle);
	b2Transform *handle_to_collider_transform(Handle handle)
};

b2BodyType body_type_to_b2_body_type(BodyType body_type);
b2Vec2 vector_to_b2_vec(Vector vector);

Vector b2_vec_to_vector(b2Vec2 vector);
} //namespace box2d
