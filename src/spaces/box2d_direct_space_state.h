#pragma once

#include <godot_cpp/classes/physics_direct_space_state2d_extension.hpp>
#include <godot_cpp/classes/physics_server2d_extension_ray_result.hpp>
#include <godot_cpp/classes/physics_server2d_extension_shape_rest_info.hpp>
#include <godot_cpp/classes/physics_server2d_extension_shape_result.hpp>

#include "box2d_space.h"

using namespace godot;

class Box2DDirectSpaceState : public PhysicsDirectSpaceState2DExtension {
	GDCLASS(Box2DDirectSpaceState, PhysicsDirectSpaceState2DExtension);

protected:
	static void _bind_methods() {}

public:
	Box2DSpace *space = nullptr;
	virtual bool _intersect_ray(const Vector2 &from, const Vector2 &to, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool hit_from_inside, PhysicsServer2DExtensionRayResult *result) override;
	virtual int32_t _intersect_point(const Vector2 &position, uint64_t canvas_instance_id, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *results, int32_t max_results) override;
	virtual int32_t _intersect_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeResult *result, int32_t max_results) override;
	virtual bool _cast_motion(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, float *closest_safe, float *closest_unsafe) override;
	virtual bool _collide_shape(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, void *results, int32_t max_results, int32_t *result_count) override;
	virtual bool _rest_info(const RID &shape_rid, const Transform2D &transform, const Vector2 &motion, double margin, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsServer2DExtensionShapeRestInfo *rest_info) override;

	PhysicsDirectSpaceState2D *get_space_state();
	~Box2DDirectSpaceState() override = default;
};
