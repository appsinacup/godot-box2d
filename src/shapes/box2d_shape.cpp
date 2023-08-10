#include "box2d_shape.h"
#include "bodies/box2d_collision_object.h"
#include "box2d_type_conversions.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_chain_shape.h>
#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_polygon_shape.h>

b2Shape *Box2DShape::get_transformed_b2Shape(ShapeInfo shape_info, Box2DCollisionObject *body) {
	b2Shape* shape = _get_transformed_b2Shape(shape_info, body);
	created_shapes.append(shape);
	if (body) {
		shape_body_map[shape] = body;
	}
	return shape;
}

void Box2DShape::reconfigure_all_b2Shapes() {
	Vector<Box2DCollisionObject *> bodies;
	for (b2Shape *shape : created_shapes) {
		auto shape_body_iterator = shape_body_map.find(shape);
		// if it has no body, don't update it
		if (shape_body_iterator) {
			bodies.append(shape_body_iterator->value);
		}
	}
	for (Box2DCollisionObject *body : bodies) {
		body->recreate_shapes();
	}
}

void Box2DShape::erase_shape(b2Shape *shape) {
	created_shapes.erase(shape);
	shape_body_map.erase(shape);
}

Box2DShape::~Box2DShape() {
	// remove body references to shapes instances
	for (auto &body : shape_body_map) {
		body.value->remove_shape(this);
	}
}
