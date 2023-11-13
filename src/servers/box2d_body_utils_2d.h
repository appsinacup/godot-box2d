#ifndef BOX2D_BODY_UTILS_2D_H
#define BOX2D_BODY_UTILS_2D_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension_motion_result.hpp>
#include <godot_cpp/variant/transform2d.hpp>

using namespace godot;

class Box2DBody2D;
class Box2DSpace2D;
class Box2DShape2D;

class Box2DBodyUtils2D {
public:
	static bool body_motion_recover(
			const Box2DSpace2D &p_space, Box2DBody2D &p_body, Transform2D &p_transform, const Vector2 &p_motion, real_t p_margin, Vector2 &p_recover_motion);

	static void cast_motion(const Box2DSpace2D &p_space, Box2DBody2D &p_body, const Transform2D &p_transform, const Vector2 &p_motion,
			bool p_collide_separation_ray, real_t contact_max_allowed_penetration, real_t p_margin, real_t &p_closest_safe,
			real_t &p_closest_unsafe, int &p_best_body_shape);

	static bool body_motion_collide(const Box2DSpace2D &p_space, Box2DBody2D &p_body, const Transform2D &p_transform,
			const Vector2 &p_motion, int p_best_body_shape, real_t p_margin, PhysicsServer2DExtensionMotionResult *p_result);
};

#endif // BOX2D_BODY_UTILS_2D_H
