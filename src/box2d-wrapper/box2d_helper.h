#include <box2d.h>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/variant/rid.hpp>

namespace box2d {

class Box2DHolder {
	godot::HashMap<b2Fixture*, b2Transform*> fixture_transforms;
public:
	inline b2Transform* handle_to_fixture_transform(b2Fixture* fixture) {return fixture_transforms.get(fixture);}
};

} //namespace box2d
