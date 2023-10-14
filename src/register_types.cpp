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
#include "joints/box2d_gear_joint_2d.h"
#include "joints/box2d_joint_custom.h"

using namespace godot;

static PhysicsServerBox2DFactory *box2d_factory = nullptr;

void initialize_physics_server_box2d_module(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			ClassDB::register_class<Box2DDirectSpaceState>();
			ClassDB::register_class<Box2DDirectBodyState>();
			ClassDB::register_class<PhysicsServerBox2D>();
			ClassDB::register_class<PhysicsServerBox2DFactory>();

			box2d_factory = memnew(PhysicsServerBox2DFactory());
			PhysicsServer2DManager::get_singleton()->register_server("Box2D", Callable(box2d_factory, "create_box2d_callback"));
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			Box2DProjectSettings::register_settings();

			ClassDB::register_class<Box2DJoint2D>(true);
			ClassDB::register_class<GearJoint2D>();
		} break;
		default: {
		} break;
	}
}

void uninitialize_physics_server_box2d_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
	memdelete(box2d_factory);
}

extern "C" {

// Initialization.

GDExtensionBool GDE_EXPORT physics_server_box2d_library_init(const GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_physics_server_box2d_module);
	init_obj.register_terminator(uninitialize_physics_server_box2d_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}
