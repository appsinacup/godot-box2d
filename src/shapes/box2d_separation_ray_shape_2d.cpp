#include "box2d_separation_ray_shape_2d.h"

void Box2DSeparationRayShape2D::set_data(const Variant &p_data) {
	Dictionary d = p_data;
	length = d["length"];
	slide_on_slope = d["slide_on_slope"];
	configure(Rect2(0, 0, 0.001, length));
}

Variant Box2DSeparationRayShape2D::get_data() const {
	Dictionary d;
	d["length"] = length;
	d["slide_on_slope"] = slide_on_slope;
	return d;
}
