#pragma once

#include <godot_cpp/variant/vector2.hpp>

using namespace godot;

class Box2DProjectSettings {
public:
	static void register_settings();

	static bool should_run_on_separate_thread();
	static int get_max_threads();
	static int get_position_iterations();
	static int get_velocity_iterations();
	static float get_scaling_factor();
	static int get_physics_fps();
	static float get_default_linear_damp();
	static float get_default_angular_damp();
	static float get_default_gravity();
	static Vector2 get_default_gravity_vector();
};
