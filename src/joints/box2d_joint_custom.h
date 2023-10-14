#pragma once

#include <godot_cpp/classes/joint2d.hpp>
#include <godot_cpp/classes/node2d.hpp>

class JoltPhysicsServer3D;

#include <godot_cpp/classes/joint2d.hpp>

#include <godot_cpp/core/class_db.hpp>

#include <type_traits>

using namespace godot;

class Box2DJoint2D : public Joint2D {
	GDCLASS(Box2DJoint2D, Joint2D)

protected:
	static void _bind_methods(){}
public:

	Box2DJoint2D(){}
};
