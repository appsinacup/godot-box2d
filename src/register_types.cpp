#include "register_types.h"

#include <godot/gdnative_interface.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/classes/physics_server2d_manager.hpp>

#include "physics_server_box2d.h"

using namespace godot;

static PhysicsServerBox2DFactory *box2d_factory = nullptr;

void initialize_physics_server_box2d_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}
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

GDNativeBool GDN_EXPORT physics_server_box2d_library_init(const GDNativeInterface *p_interface, const GDNativeExtensionClassLibraryPtr p_library, GDNativeInitialization *r_initialization) {
	godot::GDExtensionBinding::InitObject init_obj(p_interface, p_library, r_initialization);

	init_obj.register_initializer(initialize_physics_server_box2d_module);
	init_obj.register_terminator(uninitialize_physics_server_box2d_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SERVERS);

	return init_obj.init();
}
}
