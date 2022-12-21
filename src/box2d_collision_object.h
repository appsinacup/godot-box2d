#ifndef BOX2D_COLLISION_OBJECT_H
#define BOX2D_COLLISION_OBJECT_H

#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/templates/vector.hpp>

#include <box2d/b2_body.h>

#include "box2d_shape.h"
#include "box2d_space.h"
#include "box2d_type_conversions.h"

using namespace godot;

class Box2DCollisionObject {
private:
	RID self;

	b2Body *body = nullptr;
	b2BodyDef *body_def = nullptr;
	Box2DSpace* space = nullptr;

	struct Shape {
		Transform2D xform;
		Box2DShape *shape = nullptr;
		b2Fixture *fixture = nullptr;
		bool disabled = false;
	};

	Vector<Shape> shapes;
	Transform2D transform;

	void _update_shapes();

protected:
	_FORCE_INLINE_ void _set_transform(const Transform2D &p_transform, bool p_update_shapes = true) {
		transform = p_transform;
		if (body_def) {
			godot_to_box2d(transform.get_origin(), body_def->position);
			body_def->position.y *= -1; // NOTE: flip y
		}
		if (body) {
			Vector2 pos = transform.get_origin();
			b2Vec2 box2d_pos;
			godot_to_box2d(pos, box2d_pos);
			box2d_pos.y *= -1.0; // NOTE: flip y
			body->SetTransform(box2d_pos, transform.get_rotation());
		}
		if (p_update_shapes) {
			_update_shapes();
		}
	}
	_FORCE_INLINE_ void _set_transform_from_box2d() {
		ERR_FAIL_COND(!body);
		b2Vec2 box2d_pos = body->GetPosition();
		Vector2 pos;
		box2d_to_godot(box2d_pos, pos);
		pos.y *= -1.0; // NOTE: flip y
		transform = Transform2D(body->GetAngle(), pos);
	}

	void _set_space(Box2DSpace* p_space);

public:
	_FORCE_INLINE_ void set_self(const RID &p_self) { self = p_self; }
	_FORCE_INLINE_ RID get_self() const { return self; }

	b2BodyDef *get_b2BodyDef() { return body_def; }
	void set_b2BodyDef(b2BodyDef* p_body_def) { body_def = p_body_def; }
	b2Body *get_b2Body() { return body; }
	void set_b2Body(b2Body* p_body) { body = p_body; }

	void add_shape(Box2DShape *p_shape, const Transform2D &p_transform = Transform2D(), bool p_disabled = false);
	void set_shape(int p_index, Box2DShape *p_shape);
	void set_shape_transform(int p_index, const Transform2D &p_transform);

	_FORCE_INLINE_ int get_shape_count() const { return shapes.size(); }
	_FORCE_INLINE_ Box2DShape *get_shape(int p_index) const {
		CRASH_BAD_INDEX(p_index, shapes.size());
		return shapes[p_index].shape;
	}

	_FORCE_INLINE_ const Transform2D &get_shape_transform(int p_index) const {
		CRASH_BAD_INDEX(p_index, shapes.size());
		return shapes[p_index].xform;
	}

	void remove_shape(Box2DShape *p_shape);
	void remove_shape(int p_index);

	_FORCE_INLINE_ const Transform2D &get_transform() const { return transform; }

	virtual void set_space(Box2DSpace* p_space) = 0;
	_FORCE_INLINE_ Box2DSpace *get_space() const { return space; }

	Box2DCollisionObject();
	virtual ~Box2DCollisionObject();
};

#endif // BOX2D_COLLISION_OBJECT_H
