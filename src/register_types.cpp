#include "register_types.h"

#include <gdextension_interface.h>

#include <godot_cpp/classes/physics_server2d_manager.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/callable.hpp>

#include "bodies/box2d_direct_body_state.h"
#include "servers/physics_server_box2d.h"
#include "spaces/box2d_direct_space_state.h"

using namespace godot;

static PhysicsServerBox2DFactory *box2d_factory = nullptr;

void initialize_physics_server_box2d_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	ClassDB::register_class<Box2DDirectSpaceState>(true);
	ClassDB::register_class<Box2DDirectBodyState>(true);
	ClassDB::register_class<PhysicsServerBox2D>();
	ClassDB::register_class<PhysicsServerBox2DFactory>();

	box2d_factory = memnew(PhysicsServerBox2DFactory());
	PhysicsServer2DManager::get_singleton()->register_server("Box2D", Callable(box2d_factory, "create_box2d_callback"));
}

void uninitialize_physics_server_box2d_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	memdelete(box2d_factory);
}

extern "C" {

// Initialization.

GDExtensionBool GDE_EXPORT physics_server_box2d_library_init(const GDExtensionInterface *p_interface, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_interface, p_library, r_initialization);

	init_obj.register_initializer(initialize_physics_server_box2d_module);
	init_obj.register_terminator(uninitialize_physics_server_box2d_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}
