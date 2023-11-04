
env = SConscript("godot-cpp/SConstruct")

box2d_folder = "box2d/"
box2d_include = [
	"include/",
	"src/"
]
box2d_src = [
	"b2_chain_shape.cpp",
	"b2_circle_shape.cpp",
	"b2_collide_circle.cpp",
	"b2_collide_edge.cpp",
	"b2_collide_polygon.cpp",
	"b2_collision.cpp",
	"b2_distance.cpp",
	"b2_dynamic_tree.cpp",
	"b2_edge_shape.cpp",
	"b2_polygon_shape.cpp",
	"b2_time_of_impact.cpp",
	"b2_block_allocator.cpp",
	"b2_draw.cpp",
	"b2_math.cpp",
	"b2_settings.cpp",
	"b2_stack_allocator.cpp",
	"b2_timer.cpp",
	"b2_body.cpp",
	"b2_chain_circle_contact.cpp",
	"b2_chain_polygon_contact.cpp",
	"b2_circle_contact.cpp",
	"b2_contact.cpp",
	"b2_contact_manager.cpp",
	"b2_contact_solver.cpp",
	"b2_distance_joint.cpp",
	"b2_edge_circle_contact.cpp",
	"b2_edge_polygon_contact.cpp",
	"b2_fixture.cpp",
	"b2_friction_joint.cpp",
	"b2_gear_joint.cpp",
	"b2_island.cpp",
	"b2_joint.cpp",
	"b2_motor_joint.cpp",
	"b2_mouse_joint.cpp",
	"b2_polygon_circle_contact.cpp",
	"b2_polygon_contact.cpp",
	"b2_prismatic_joint.cpp",
	"b2_pulley_joint.cpp",
	"b2_revolute_joint.cpp",
	"b2_weld_joint.cpp",
	"b2_wheel_joint.cpp",
	"b2_world.cpp",
	"b2_world_callbacks.cpp",
	"b2_rope.cpp",
]
env.Prepend(CPPPATH=[box2d_folder + folder for folder in box2d_include])

# For the reference:
# - CCFLAGS are compilation flags shared between C and C++
# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/"])
sources = [Glob("src/*.cpp"),Glob("src/bodies/*.cpp"),Glob("src/joints/*.cpp"),Glob("src/servers/*.cpp"),Glob("src/shapes/*.cpp"),Glob("src/spaces/*.cpp"),Glob("src/box2d-wrapper/*.cpp")]
sources.extend([box2d_folder + 'src/' + box2d_src_file for box2d_src_file in box2d_src])

if env["platform"] == "macos":
	library = env.SharedLibrary(
		"bin/addons/godot-box2d/bin/libgodot-box2d.{}.{}.framework/libgodot-box2d.{}.{}".format(
			env["platform"], env["target"], env["platform"], env["target"]
		),
		source=sources,
	)
else:
	library = env.SharedLibrary(
		"bin/addons/godot-box2d/bin/libgodot-box2d{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
		source=sources,
	)
Default(library)
