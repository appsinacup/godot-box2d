#ifndef BOX2D_INCLUDE_H
#define BOX2D_INCLUDE_H

#include "box2d-wrapper/box2d_wrapper.h"

#include <godot_cpp/templates/hashfuncs.hpp>
#include <godot_cpp/variant/transform2d.hpp>

using namespace godot;

namespace box2d {

inline uint32_t handle_hash(b2WorldId handle) {
	uint64_t combined = uint64_t(handle.index) + (uint64_t(handle.revision) << 32);
	return hash_one_uint64(combined);
}
inline uint32_t handle_hash(b2ShapeId handle) {
	uint64_t combined = uint64_t(handle.index) + (uint64_t(handle.world) << 32) + (uint64_t(handle.revision) << 48);
	return hash_one_uint64(combined);
}

inline uint64_t handle_pair_hash(b2WorldId handle1, b2WorldId handle2) {
	uint64_t hash1 = handle_hash(handle1);
	uint64_t hash2 = handle_hash(handle2);
	return hash1 + (hash2 << 32);
}
inline uint64_t handle_pair_hash(b2ShapeId handle1, b2ShapeId handle2) {
	uint64_t hash1 = handle_hash(handle1);
	uint64_t hash2 = handle_hash(handle2);
	return hash1 + (hash2 << 32);
}

inline ShapeInfo shape_info_from_body_shape(box2d::ShapeHandle shape_handle, const Transform2D &body_transform, const Transform2D &transform) {
	return ShapeInfo{
		shape_handle,
		body_transform,
		transform,
	};
}

} //namespace box2d

#endif // BOX2D_INCLUDE_H
