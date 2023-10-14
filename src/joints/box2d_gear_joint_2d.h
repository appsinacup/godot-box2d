#pragma once

#include <godot_cpp/classes/joint2d.hpp>
#include <godot_cpp/classes/node2d.hpp>

class JoltPhysicsServer3D;

#include "box2d_joint_custom.h"

#include <godot_cpp/core/class_db.hpp>

#include <type_traits>

using namespace godot;

class GearJoint2D : public Box2DJoint2D {
	GDCLASS(GearJoint2D, Box2DJoint2D)

protected:
	static void _bind_methods(){}
public:

	GearJoint2D(){}
};
