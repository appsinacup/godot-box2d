#include "box2d_shape_separation_ray.h"
#include "../box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_body.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShapeSeparationRay::set_data(const Variant &p_data) {
	// todo
	configured = true;
}

Variant Box2DShapeSeparationRay::get_data() const {
	// todo
	return Variant();
}

b2Shape *Box2DShapeSeparationRay::get_transformed_b2Shape(int p_index, const Transform2D &p_transform, bool one_way, bool is_static) {
	ERR_FAIL_INDEX_V(p_index, 1, nullptr);
	b2EdgeShape *shape = memnew(b2EdgeShape);
	b2Vec2 edge_endpoints[2];
	godot_to_box2d(p_transform.xform(a), edge_endpoints[0]);
	godot_to_box2d(p_transform.xform(b), edge_endpoints[1]);
	if (one_way) {
		b2Vec2 dirV0 = edge_endpoints[0] - edge_endpoints[1];
		shape->SetOneSided(edge_endpoints[1] + dirV0, edge_endpoints[0], edge_endpoints[1], edge_endpoints[0] - dirV0);
	} else {
		shape->SetTwoSided(edge_endpoints[0], edge_endpoints[1]);
	}
	return shape;
}
