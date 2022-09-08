#ifndef PHYSICS_SERVER_BOX2D_H
#define PHYSICS_SERVER_BOX2D_H

// We don't need windows.h in this plugin but many others do and it throws up on itself all the time
// So best to include it and make sure CI warns us when we use something Microsoft took for their own goals....
#ifdef WIN32
#include <windows.h>
#endif

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/physics_server2d_extension.hpp>

#include <godot_cpp/core/binder_common.hpp>

using namespace godot;

class PhysicsServerBox2D : public PhysicsServer2DExtension
{
	GDCLASS(PhysicsServerBox2D, PhysicsServer2DExtension);

protected:
	static void _bind_methods() {};

public:
	virtual void _set_active(bool active) override;
	virtual void _init() override;
	virtual void _step(double step) override;
	virtual void _sync() override;
	virtual void _end_sync() override;
	virtual void _finish() override;

	PhysicsServerBox2D();
	~PhysicsServerBox2D();
};

class PhysicsServerBox2DFactory: public Object {
	GDCLASS(PhysicsServerBox2DFactory, Object);

protected:
	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("create_box2d_callback"), &PhysicsServerBox2DFactory::_create_box2d_callback);
	}

public:
	PhysicsServer2D *_create_box2d_callback() {
		PhysicsServer2D *physics_server_2d = memnew(PhysicsServerBox2D());
		return physics_server_2d;
	}

};

#endif // ! PHYSICS_SERVER_BOX2D_H
