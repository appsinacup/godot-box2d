#include "box2d_project_settings.h"

#include <godot_cpp/classes/project_settings.hpp>

using namespace godot;

constexpr char DEFAULT_LINEAR_DAMP[] = "physics/2d/default_linear_damp";
constexpr char DEFAULT_ANGULAR_DAMP[] = "physics/2d/default_angular_damp";
constexpr char DEFAULT_GRAVITY_VECTOR[] = "physics/2d/default_gravity_vector";
constexpr char DEFAULT_GRAVITY[] = "physics/2d/default_gravity";

constexpr char PHYSICS_FPS[] = "physics/common/physics_ticks_per_second";
constexpr char RUN_ON_SEPARATE_THREAD[] = "physics/2d/run_on_separate_thread";
constexpr char MAX_THREADS[] = "threading/worker_pool/max_threads";
constexpr char POSITION_ITERATIONS[] = "physics/box_2d/solver/position_iterations";
constexpr char VELOCITY_ITERATIONS[] = "physics/box_2d/solver/velocity_iterations";
constexpr char SCALING_FACTOR[] = "physics/box_2d/scaling_factor";

void register_setting(
		const String &p_name,
		const Variant &p_value,
		bool p_needs_restart,
		PropertyHint p_hint,
		const String &p_hint_string) {
	ProjectSettings *project_settings = ProjectSettings::get_singleton();

	if (!project_settings->has_setting(p_name)) {
		project_settings->set(p_name, p_value);
	}

	Dictionary property_info;
	property_info["name"] = p_name;
	property_info["type"] = p_value.get_type();
	property_info["hint"] = p_hint;
	property_info["hint_string"] = p_hint_string;

	project_settings->add_property_info(property_info);
	project_settings->set_initial_value(p_name, p_value);
	project_settings->set_restart_if_changed(p_name, p_needs_restart);

	// HACK(mihe): We want our settings to appear in the order we register them in, but if we start
	// the order at 0 we end up moving the entire `physics/` group to the top of the tree view, so
	// instead we give it a hefty starting order and increment from there, which seems to give us
	// the desired effect.
	static int32_t order = 1000000;

	project_settings->set_order(p_name, order++);
}

void register_setting_plain(
		const String &p_name,
		const Variant &p_value,
		bool p_needs_restart = false) {
	register_setting(p_name, p_value, p_needs_restart, PROPERTY_HINT_NONE, {});
}

void register_setting_hinted(
		const String &p_name,
		const Variant &p_value,
		const String &p_hint_string,
		bool p_needs_restart = false) {
	register_setting(p_name, p_value, p_needs_restart, PROPERTY_HINT_NONE, p_hint_string);
}

void register_setting_ranged(
		const String &p_name,
		const Variant &p_value,
		const String &p_hint_string,
		bool p_needs_restart = false) {
	register_setting(p_name, p_value, p_needs_restart, PROPERTY_HINT_RANGE, p_hint_string);
}

void Box2DProjectSettings::register_settings() {
	register_setting_ranged(VELOCITY_ITERATIONS, 8, U"2,16,or_greater");
	register_setting_ranged(POSITION_ITERATIONS, 3, U"1,16,or_greater");
	register_setting_ranged(SCALING_FACTOR, 100.0f, U"1,100,or_greater");
}

template <typename TType>
TType get_setting(const char *p_setting) {
	const ProjectSettings *project_settings = ProjectSettings::get_singleton();
	const Variant setting_value = project_settings->get_setting_with_override(p_setting);
	const Variant::Type setting_type = setting_value.get_type();
	const Variant::Type expected_type = Variant(TType()).get_type();

	ERR_FAIL_COND_V(setting_type != expected_type, Variant());

	return setting_value;
}

bool Box2DProjectSettings::should_run_on_separate_thread() {
	return get_setting<bool>(RUN_ON_SEPARATE_THREAD);
}

int Box2DProjectSettings::get_max_threads() {
	return get_setting<int>(MAX_THREADS);
}

int Box2DProjectSettings::get_position_iterations() {
	return get_setting<int>(POSITION_ITERATIONS);
}

int Box2DProjectSettings::get_velocity_iterations() {
	return get_setting<int>(VELOCITY_ITERATIONS);
}

int Box2DProjectSettings::get_physics_fps() {
	return get_setting<int>(PHYSICS_FPS);
}

float Box2DProjectSettings::get_default_linear_damp() {
	return get_setting<float>(DEFAULT_LINEAR_DAMP);
}
float Box2DProjectSettings::get_default_angular_damp() {
	return get_setting<float>(DEFAULT_ANGULAR_DAMP);
}
float Box2DProjectSettings::get_default_gravity() {
	return get_setting<float>(DEFAULT_GRAVITY);
}
Vector2 Box2DProjectSettings::get_default_gravity_vector() {
	return get_setting<Vector2>(DEFAULT_GRAVITY_VECTOR);
}
