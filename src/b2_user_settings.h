#ifndef B2_USER_SETTINGS_H
#define B2_USER_SETTINGS_H

#include <stdarg.h>
#include <stdint.h>

#include <godot_cpp/core/memory.hpp>

// Tunable Constants

/// You can use this to change the length scale used by your game.
/// For example for inches you could use 39.4.
#define b2_lengthUnitsPerMeter 1.0f

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
#define b2_maxPolygonVertices 8

// User data

class Box2DCollisionObject;

struct B2_API b2BodyUserData {
	b2BodyUserData() {
		collision_object = nullptr;
	}

	Box2DCollisionObject *collision_object;
};

struct B2_API b2FixtureUserData {
	b2FixtureUserData() {
		shape_idx = 0;
		box2d_fixture_idx = 0;
	}

	int shape_idx;
	int box2d_fixture_idx;
};

/// You can define this to inject whatever data you want in b2Joint
struct B2_API b2JointUserData {
	b2JointUserData() {
		pointer = 0;
	}

	/// For legacy compatibility
	uintptr_t pointer;
};

// Memory Allocation using Godot's functions

inline void *b2Alloc(int32 size) {
	return memalloc(size);
}

inline void b2Free(void *mem) {
	memfree(mem);
}

/// Default logging function
B2_API void b2Log_Default(const char *string, va_list args);

/// Implement this to use your own logging.
inline void b2Log(const char *string, ...) {
	va_list args;
	va_start(args, string);
	b2Log_Default(string, args);
	va_end(args);
}

#endif // B2_USER_SETTINGS_H
