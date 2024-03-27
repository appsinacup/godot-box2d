#pragma once

#include <godot_cpp/variant/vector2.hpp>

using namespace godot;

class Box2DProjectSettings {
public:
	static void register_settings();

	static bool should_run_on_separate_thread();
	static int get_max_threads();
	static bool get_logging_enabled();
	static int get_sub_step_count();
};
