#pragma once

#include <godot_cpp/core/class_db.hpp>
using namespace godot;

void initialize_physics_server_box2d_module(ModuleInitializationLevel p_level);
void uninitialize_physics_server_box2d_module(ModuleInitializationLevel p_level);
