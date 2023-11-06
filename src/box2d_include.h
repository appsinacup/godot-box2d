#ifndef BOX2D_INCLUDE_H
#define BOX2D_INCLUDE_H

#include "b2_user_settings.h"
#include "box2d-wrapper/box2d_wrapper.h"

#include <godot_cpp/templates/hashfuncs.hpp>
#include <godot_cpp/variant/transform2d.hpp>

using namespace godot;

namespace box2d {

inline uint32_t handle_hash(b2World *handle) {
	return hash_one_uint64(uint64_t(handle));
	//uint64_t combined = uint64_t(handle.id) + (uint64_t(handle.generation) << 32);
	//return hash_one_uint64(combined);
}
inline uint32_t handle_hash(b2Fixture *handle) {
	return hash_one_uint64(uint64_t(handle));
	//uint64_t combined = uint64_t(handle.id) + (uint64_t(handle.generation) << 32);
	//return hash_one_uint64(combined);
}

inline uint64_t handle_pair_hash(b2World *handle1, b2World *handle2) {
	uint64_t hash1 = handle_hash(handle1);
	uint64_t hash2 = handle_hash(handle2);
	return hash1 + (hash2 << 32);
}
inline uint64_t handle_pair_hash(b2Fixture *handle1, b2Fixture *handle2) {
	uint64_t hash1 = handle_hash(handle1);
	uint64_t hash2 = handle_hash(handle2);
	return hash1 + (hash2 << 32);
}

inline ShapeInfo shape_info_from_body_shape(b2Shape *shape_handle, const Transform2D &transform) {
	return ShapeInfo{
		shape_handle,
		transform,
	};
}

} //namespace box2d

#endif // BOX2D_INCLUDE_H
