#ifndef BOX2D_SHAPE_2D_H
#define BOX2D_SHAPE_2D_H

#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/local_vector.hpp>

#include "../box2d_include.h"

using namespace godot;

class Box2DShape2D;

class Box2DShapeOwner2D {
public:
	virtual void _shape_changed(Box2DShape2D *p_shape) = 0;
	virtual void remove_shape(Box2DShape2D *p_shape) = 0;

	virtual ~Box2DShapeOwner2D() {}
};

class Box2DShape2D {
	RID rid;
	Rect2 aabb;
	bool configured = false;

	HashMap<Box2DShapeOwner2D *, int> owners;

	box2d::Handle shape_handle = box2d::invalid_handle();

protected:
	void configure(const Rect2 &p_aabb);

	virtual box2d::Handle create_box2d_shape() const = 0;
	void destroy_box2d_shape();

public:
	_FORCE_INLINE_ void set_rid(const RID &p_rid) { rid = p_rid; }
	_FORCE_INLINE_ RID get_rid() const { return rid; }

	virtual PhysicsServer2D::ShapeType get_type() const = 0;

	virtual void apply_box2d_transform(box2d::Vector &position, real_t &angle) const {}

	virtual bool allows_one_way_collision() const { return true; }

	box2d::Handle get_box2d_shape();

	_FORCE_INLINE_ Rect2 get_aabb(Vector2 origin = Vector2()) const {
		Rect2 aabb_clone = aabb;
		aabb_clone.position += origin;
		return aabb_clone;
	}
	_FORCE_INLINE_ bool is_configured() const { return configured; }

	void add_owner(Box2DShapeOwner2D *p_owner);
	void remove_owner(Box2DShapeOwner2D *p_owner);
	bool is_owner(Box2DShapeOwner2D *p_owner) const;
	const HashMap<Box2DShapeOwner2D *, int> &get_owners() const;

	virtual void set_data(const Variant &p_data) = 0;
	virtual Variant get_data() const = 0;

	virtual real_t get_moment_of_inertia(real_t p_mass, const Size2 &p_scale) const = 0;

	Box2DShape2D() {}
	virtual ~Box2DShape2D();
};

#endif // BOX2D_SHAPE_2D_H
