#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

box2d_folder = "box2d/"
box2d_include = [
	"include/",
	"src/"
]
box2d_src = [
	"collision/b2_broad_phase.cpp",
	"collision/b2_chain_shape.cpp",
	"collision/b2_circle_shape.cpp",
	"collision/b2_collide_circle.cpp",
	"collision/b2_collide_edge.cpp",
	"collision/b2_collide_polygon.cpp",
	"collision/b2_collision.cpp",
	"collision/b2_distance.cpp",
	"collision/b2_dynamic_tree.cpp",
	"collision/b2_edge_shape.cpp",
	"collision/b2_polygon_shape.cpp",
	"collision/b2_time_of_impact.cpp",
	"common/b2_block_allocator.cpp",
	"common/b2_draw.cpp",
	"common/b2_math.cpp",
	"common/b2_settings.cpp",
	"common/b2_stack_allocator.cpp",
	"common/b2_timer.cpp",
	"dynamics/b2_body.cpp",
	"dynamics/b2_chain_circle_contact.cpp",
	"dynamics/b2_chain_polygon_contact.cpp",
	"dynamics/b2_circle_contact.cpp",
	"dynamics/b2_contact.cpp",
	"dynamics/b2_contact_manager.cpp",
	"dynamics/b2_contact_solver.cpp",
	"dynamics/b2_distance_joint.cpp",
	"dynamics/b2_edge_circle_contact.cpp",
	"dynamics/b2_edge_polygon_contact.cpp",
	"dynamics/b2_fixture.cpp",
	"dynamics/b2_friction_joint.cpp",
	"dynamics/b2_gear_joint.cpp",
	"dynamics/b2_island.cpp",
	"dynamics/b2_joint.cpp",
	"dynamics/b2_motor_joint.cpp",
	"dynamics/b2_mouse_joint.cpp",
	"dynamics/b2_polygon_circle_contact.cpp",
	"dynamics/b2_polygon_contact.cpp",
	"dynamics/b2_prismatic_joint.cpp",
	"dynamics/b2_pulley_joint.cpp",
	"dynamics/b2_revolute_joint.cpp",
	"dynamics/b2_weld_joint.cpp",
	"dynamics/b2_wheel_joint.cpp",
	"dynamics/b2_world.cpp",
	"dynamics/b2_world_callbacks.cpp",
	"rope/b2_rope.cpp",
]

env.Prepend(CPPPATH=[box2d_folder + folder for folder in box2d_include])

# For the reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

# Make Box2D include "b2_user_settings.h"
env.Append(CPPDEFINES="B2_USER_SETTINGS")

# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/"])
sources = [Glob("src/*.cpp"),Glob("src/bodies/*.cpp"),Glob("src/joints/*.cpp"),Glob("src/servers/*.cpp"),Glob("src/shapes/*.cpp"),Glob("src/spaces/*.cpp")]
sources.extend([box2d_folder + 'src/' + box2d_src_file for box2d_src_file in box2d_src])

if env["platform"] == "macos":
	library = env.SharedLibrary(
		"demo/bin/libphysics_server_box2d.{}.{}.framework/libphysics_server_box2d.{}.{}".format(
			env["platform"], env["target"], env["platform"], env["target"]
		),
		source=sources,
	)
else:
	library = env.SharedLibrary(
		"demo/bin/libphysics_server_box2d{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
		source=sources,
	)

Default(library)
