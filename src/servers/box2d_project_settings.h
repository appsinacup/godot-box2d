#pragma once

class Box2DProjectSettings {
public:
	static void register_settings();

	static bool should_run_on_separate_thread();
	static int get_max_threads();
	static int get_position_iterations();
	static int get_velocity_iterations();
};
