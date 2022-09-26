#include "box2d_collision_object.h"

#include <godot_cpp/core/memory.hpp>

#include <box2d/b2_fixture.h>

void Box2DCollisionObject::_update_shapes() {
	if (!space) {
		return;
	}

	for (int i = 0; i < shapes.size(); i++) {
		Shape &s = shapes.write[i];
		if (s.disabled) {
			continue;
		}

		//not quite correct, should compute the next matrix..
		Transform2D xform = transform * s.xform;

		if (!s.fixture) {
			b2FixtureDef fixture_def;
			fixture_def.shape = s.shape->get_b2Shape();
			// TODO: use xform here
			fixture_def.density = 1.0f;
			s.fixture = body->CreateFixture(&fixture_def);
		}
		// TODO: use i?

		//space->get_broadphase()->move(s.bpid, shape_aabb);
	}
}

void Box2DCollisionObject::add_shape(Box2DShape *p_shape, const Transform2D &p_transform, bool p_disabled) {
	Shape s;
	s.shape = p_shape;
	s.xform = p_transform;
	s.disabled = p_disabled;
	shapes.push_back(s);

	// TODO (queue) update
}

void Box2DCollisionObject::set_shape(int p_index, Box2DShape *p_shape) {
	ERR_FAIL_INDEX(p_index, shapes.size());
	//shapes[p_index].shape->remove_owner(this);
	shapes.write[p_index].shape = p_shape;

	// TODO: (queue) update
}

void Box2DCollisionObject::set_shape_transform(int p_index, const Transform2D &p_transform) {
	ERR_FAIL_INDEX(p_index, shapes.size());

	shapes.write[p_index].xform = p_transform;

	// TODO: (queue) update
}

void Box2DCollisionObject::remove_shape(Box2DShape *p_shape) {
	//remove a shape, all the times it appears
	for (int i = 0; i < shapes.size(); i++) {
		if (shapes[i].shape == p_shape) {
			remove_shape(i);
			i--;
		}
	}
}

void Box2DCollisionObject::remove_shape(int p_index) {
	//remove anything from shape to be erased to end, so subindices don't change
	ERR_FAIL_INDEX(p_index, shapes.size());
	for (int i = p_index; i < shapes.size(); i++) {
		if (!shapes[i].fixture) {
			continue;
		}
		//should never get here with a null owner
		body->DestroyFixture(shapes[i].fixture);
		shapes.write[i].fixture = nullptr;
	}
	shapes.remove_at(p_index);

	// TODO: (queue) update
}

void Box2DCollisionObject::_set_space(Box2DSpace* p_space) {
	if (space) {
		for (int i = 0; i < shapes.size(); i++) {
			Shape &s = shapes.write[i];
			if (s.fixture) {
				body->DestroyFixture(shapes[i].fixture);
				s.fixture = nullptr;
			}
		}
		space->remove_object(this);
	}
	space = p_space;
	if (space) {
		space->add_object(this);
		_update_shapes();
	}
}

Box2DCollisionObject::Box2DCollisionObject() {
}

Box2DCollisionObject::~Box2DCollisionObject() {
	if (body_def) {
		memdelete(body_def);
	}
}
