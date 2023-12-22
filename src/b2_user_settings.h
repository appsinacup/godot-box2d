#ifndef B2_USER_SETTINGS_H
#define B2_USER_SETTINGS_H

#include <stdarg.h>
#include <stdint.h>

#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/transform2d.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include <box2d/box2d.h>

// Tunable Constants

// You can use this to change the length scale used by your game.
// For example for inches you could use 39.4.
#define b2_lengthUnitsPerMeter 100.0f

// The maximum number of vertices on a convex polygon. You cannot increase
// this too much because b2BlockAllocator has a maximum object size.
#define b2_maxPolygonVertices 64

class Box2DCollisionObject2D;
class Box2DShape2D;

// You can define this to inject whatever data you want in b2Body
struct b2BodyUserData {
	b2BodyUserData() :
			old_linear_velocity(0, 0), old_angular_velocity(0), constant_force(0, 0), constant_torque(0), collision_object(nullptr) {}

	// for kinematic body
	godot::Vector2 old_linear_velocity;
	real_t old_angular_velocity;
	godot::Vector2 constant_force;
	real_t constant_torque;
	Box2DCollisionObject2D *collision_object;
};

// You can define this to inject whatever data you want in b2Fixture
struct b2FixtureUserData {
	b2FixtureUserData() :
			shape_idx(-1), transform(), collision_object(nullptr) {}

	godot::Transform2D transform;
	int shape_idx;
	Box2DCollisionObject2D *collision_object;
};

// Memory Allocation using Godot's functions
inline void *b2AllocGodot(uint32_t size) {
	return memalloc(size);
}

inline void b2FreeGodot(void *mem) {
	memfree(mem);
}

inline int b2AssertFcnGodot(const char* condition, const char* fileName, int lineNumber)
{
	ERR_PRINT("Box2D assert: " + String(condition) + " " + String(fileName) + " line: " + rtos(lineNumber));
	// don't assert it, just print error.
	return 0;
}

#endif // B2_USER_SETTINGS_H
