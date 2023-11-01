#include "register_types.h"

#include <gdextension_interface.h>

#include <godot_cpp/classes/physics_server2d_manager.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/callable.hpp>

#include "bodies/box2d_direct_body_state_2d.h"
#include "servers/box2d_physics_server_2d.h"
#include "servers/box2d_project_settings.h"
#include "spaces/box2d_direct_space_state_2d.h"
#include "spaces/box2d_space_2d.h"

#if defined(WINDOWS_ENABLED)
// Libs needed to link Box2D
#pragma comment(lib, "WS2_32")
#pragma comment(lib, "BCrypt")
#pragma comment(lib, "userenv")
#pragma comment(lib, "advapi32")
#pragma comment(lib, "Ntdll")
#endif

using namespace godot;

static Box2DPhysicsServer2DFactory *box2d_factory = nullptr;

void initialize_box2d_module(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			ClassDB::register_class<Box2DDirectBodyState2D>(true);
			ClassDB::register_class<Box2DDirectSpaceState2D>(true);
			ClassDB::register_class<Box2DPhysicsServer2D>();
			ClassDB::register_class<Box2DPhysicsServer2DFactory>();

			box2d_factory = memnew(Box2DPhysicsServer2DFactory());
			PhysicsServer2DManager::get_singleton()->register_server("Box2D", Callable(box2d_factory, "create_box2d_callback"));
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			Box2DProjectSettings::register_settings();
		} break;
		default: {
		} break;
	}
}

void uninitialize_box2d_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	memdelete(box2d_factory);
}

extern "C" {

// Initialization.

GDExtensionBool GDE_EXPORT physics_server_box2d_library_init(const GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_box2d_module);
	init_obj.register_terminator(uninitialize_box2d_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}
