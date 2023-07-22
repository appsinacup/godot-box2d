#include "box2d_shape.h"
#include "bodies/box2d_collision_object.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

void Box2DShape::set_body(Box2DCollisionObject *p_body) {
	body = p_body;
}
Box2DCollisionObject *Box2DShape::get_body() const {
	return body;
}
