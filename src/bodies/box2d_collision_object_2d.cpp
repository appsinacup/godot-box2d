#include "box2d_collision_object_2d.h"

#include "../servers/box2d_physics_server_2d.h"
#include "../spaces/box2d_space_2d.h"

void Box2DCollisionObject2D::add_shape(Box2DShape2D *p_shape, const Transform2D &p_transform, bool p_disabled) {
	Shape shape;
	shape.shape = p_shape;
	shape.xform = p_transform;
	shape.disabled = p_disabled;
	shape.one_way_collision = false;
	shape.one_way_collision_margin = 0;

	if (!shape.disabled) {
		_create_shape(shape, shapes.size());
		_update_shape_transform(shape);
	}

	shapes.push_back(shape);
	p_shape->add_owner(this);

	//if (!pending_shape_update_list.in_list()) {
	//	Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void Box2DCollisionObject2D::set_shape(int p_index, Box2DShape2D *p_shape) {
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Shape &shape = shapes[p_index];

	_destroy_shape(shape, p_index);

	shape.shape->remove_owner(this);
	shape.shape = p_shape;

	p_shape->add_owner(this);

	if (!shape.disabled) {
		_create_shape(shape, p_index);
		_update_shape_transform(shape);
	}

	//if (!pending_shape_update_list.in_list()) {
	//	Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void Box2DCollisionObject2D::set_shape_transform(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Shape &shape = shapes[p_index];
	shape.xform = p_transform;

	_update_shape_transform(shape);

	//if (!pending_shape_update_list.in_list()) {
	//	Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void Box2DCollisionObject2D::set_shape_disabled(int p_index, bool p_disabled) {
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Box2DCollisionObject2D::Shape &shape = shapes[p_index];
	if (shape.disabled == p_disabled) {
		return;
	}

	shape.disabled = p_disabled;

	if (shape.disabled) {
		_destroy_shape(shape, p_index);
	} else {
		_create_shape(shape, p_index);
		_update_shape_transform(shape);
	}

	// if (p_disabled && shape.bpid != 0) {
	// 	space->get_broadphase()->remove(shape.bpid);
	// 	shape.bpid = 0;
	// 	if (!pending_shape_update_list.in_list()) {
	// 		Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	// 	}
	// } else if (!p_disabled && shape.bpid == 0) {
	// 	if (!pending_shape_update_list.in_list()) {
	// 		Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	// 	}
	// }

	//if (!pending_shape_update_list.in_list()) {
	//	Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void Box2DCollisionObject2D::remove_shape(Box2DShape2D *p_shape) {
	//remove a shape, all the times it appears
	for (uint32_t i = 0; i < shapes.size(); i++) {
		if (shapes[i].shape == p_shape) {
			remove_shape(i);
			i--;
		}
	}
}

void Box2DCollisionObject2D::remove_shape(int p_index) {
	//remove anything from shape to be erased to end, so subindices don't change
	ERR_FAIL_INDEX(p_index, (int)shapes.size());

	Shape &shape = shapes[p_index];

	if (!shape.disabled) {
		_destroy_shape(shape, p_index);
	}

	shape.shape->remove_owner(this);
	shapes.remove_at(p_index);

	//if (!pending_shape_update_list.in_list()) {
	//	Box2DPhysicsServer2D::singleton->pending_shape_update_list.add(&pending_shape_update_list);
	//}

	if (space) {
		_shapes_changed();
	}
}

void Box2DCollisionObject2D::_unregister_shapes() {
}

void Box2DCollisionObject2D::_update_transform() {
	if (!space) {
		return;
	}

	b2World *space_handle = space->get_handle();
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!box2d::is_handle_valid(body_handle));

	b2Vec2 position = box2d::body_get_position(space_handle, body_handle);
	real_t angle = box2d::body_get_angle(space_handle, body_handle);

	transform.set_origin(Vector2(position.x, position.y));
	transform.set_rotation(angle);

	inv_transform = transform.affine_inverse();
}

void Box2DCollisionObject2D::set_transform(const Transform2D &p_transform, bool wake_up) {
	transform = p_transform;
	inv_transform = transform.affine_inverse();

	if (space) {
		b2World *space_handle = space->get_handle();
		ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

		ERR_FAIL_COND(!box2d::is_handle_valid(body_handle));

		const Vector2 &origin = transform.get_origin();
		b2Vec2 position = { origin.x, origin.y };
		real_t rotation = transform.get_rotation();
		box2d::body_set_transform(space_handle, body_handle, position, rotation, wake_up, space->get_last_step());

		for (uint32_t i = 0; i < shapes.size(); i++) {
			Shape &shape = shapes[i];
			if (shape.disabled) {
				continue;
			}

			_update_shape_transform(shape);
		}
	}
}

void Box2DCollisionObject2D::_create_shape(Shape &shape, uint32_t p_shape_index) {
	if (!space) {
		return;
	}

	b2World *space_handle = space->get_handle();
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(box2d::is_handle_valid(shape.collider_handle));

	box2d::Material mat = box2d::default_material();
	_init_material(mat);

	box2d::ShapeHandle shape_handle = shape.shape->get_box2d_shape();
	ERR_FAIL_COND(!box2d::is_handle_valid(shape_handle));

	b2FixtureUserData user_data;
	set_collider_user_data(user_data, p_shape_index);

	switch (type) {
		case TYPE_BODY: {
			shape.collider_handle = box2d::collider_create_solid(space_handle, shape_handle, &mat, body_handle, user_data);
		} break;
		case TYPE_AREA: {
			shape.collider_handle = box2d::collider_create_sensor(space_handle, shape_handle, body_handle, user_data);
		} break;
	}

	ERR_FAIL_COND(!box2d::is_handle_valid(shape.collider_handle));
	_init_collider(shape.collider_handle);
}

void Box2DCollisionObject2D::_destroy_shape(Shape &shape, uint32_t p_shape_index) {
	if (!space) {
		return;
	}

	b2World *space_handle = space->get_handle();
	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

	ERR_FAIL_COND(!box2d::is_handle_valid(shape.collider_handle));

	if (area_detection_counter > 0) {
		// Keep track of body information for delayed removal
		for (int i = 0; i < shape.collider_handle.count; i++) {
			space->add_removed_collider(shape.collider_handle.handles[i], this, p_shape_index);
		}
	}
	box2d::collider_destroy(space_handle, shape.collider_handle);
	shape.collider_handle = box2d::invalid_fixture_handle(); // collider_handle = box2d ID
}

void Box2DCollisionObject2D::_update_shape_transform(const Shape &shape) {
	if (!space) {
		return;
	}

	b2World *space_handle = space->get_handle();

	ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

	box2d::ShapeInfo shape_info{
		shape.shape->get_box2d_shape(),
		transform,
		shape.xform,
	};
	box2d::collider_set_transform(space_handle, shape.collider_handle, shape_info);
}

void Box2DCollisionObject2D::_set_space(Box2DSpace2D *p_space) {
	if (space) {
		b2World *space_handle = space->get_handle();
		ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

		ERR_FAIL_COND(!box2d::is_handle_valid(body_handle));

		for (uint32_t i = 0; i < shapes.size(); i++) {
			Shape &shape = shapes[i];
			if (shape.disabled) {
				continue;
			}

			_destroy_shape(shape, i);
		}
		// This call also destroys the colliders
		box2d::body_destroy(space_handle, body_handle);
		body_handle = box2d::invalid_body_handle();

		// Reset area detection counter to keep it consistent for new detections
		area_detection_counter = 0;
	}

	space = p_space;

	if (space) {
		b2World *space_handle = space->get_handle();
		ERR_FAIL_COND(!box2d::is_handle_valid(space_handle));

		ERR_FAIL_COND(box2d::is_handle_valid(body_handle));

		b2BodyUserData user_data;
		set_body_user_data(user_data);

		b2Vec2 position = { transform.get_origin().x, transform.get_origin().y };
		real_t angle = transform.get_rotation();
		if (mode == PhysicsServer2D::BODY_MODE_STATIC) {
			body_handle = box2d::body_create(space_handle, position, angle, user_data, b2BodyType::b2_kinematicBody);
		} else if (mode == PhysicsServer2D::BODY_MODE_KINEMATIC) {
			body_handle = box2d::body_create(space_handle, position, angle, user_data, b2BodyType::b2_kinematicBody);
		} else {
			body_handle = box2d::body_create(space_handle, position, angle, user_data, b2BodyType::b2_dynamicBody);
		}
		if (type == TYPE_AREA) {
			box2d::body_set_can_sleep(space_handle, body_handle, false);
			box2d::body_set_gravity_scale(space_handle, body_handle, 0.0, true);
		}

		for (uint32_t i = 0; i < shapes.size(); i++) {
			Shape &shape = shapes[i];
			if (shape.disabled) {
				continue;
			}

			_create_shape(shape, i);
			_update_shape_transform(shape);
		}
	}
}

void Box2DCollisionObject2D::set_body_user_data(b2BodyUserData &r_user_data) {
	r_user_data.collision_object = this;
}

Box2DCollisionObject2D *Box2DCollisionObject2D::get_body_user_data(const b2BodyUserData &p_user_data) {
	return (Box2DCollisionObject2D *)p_user_data.collision_object;
}

void Box2DCollisionObject2D::set_collider_user_data(b2FixtureUserData &r_user_data, uint32_t p_shape_index) {
	r_user_data.collision_object = this;
	r_user_data.shape_idx = p_shape_index;
}

Box2DCollisionObject2D *Box2DCollisionObject2D::get_collider_user_data(const b2FixtureUserData &p_user_data, uint32_t &r_shape_index) {
	r_shape_index = (uint32_t)p_user_data.shape_idx;
	return (Box2DCollisionObject2D *)p_user_data.collision_object;
}

void Box2DCollisionObject2D::_shape_changed(Box2DShape2D *p_shape) {
	if (!space) {
		return;
	}

	for (uint32_t i = 0; i < shapes.size(); i++) {
		Shape &shape = shapes[i];
		if (shape.shape != p_shape) {
			continue;
		}
		if (shape.disabled) {
			continue;
		}

		_destroy_shape(shape, i);

		_create_shape(shape, i);
		_update_shape_transform(shape);
	}

	_shapes_changed();
}

Box2DCollisionObject2D::Box2DCollisionObject2D(Type p_type) {
	//: pending_shape_update_list(this) {
	type = p_type;
}
