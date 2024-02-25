#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

env.Append(
	CPPDEFINES=[
		"BOX2D_LENGTH_UNIT_PER_METER=100.0",
		"BOX2D_MAX_POLYGON_VERTICES=64",
		"BOX2D_AVX2=ON"
	]
)

env.Prepend(CPPPATH=["box2d/extern/simde", "box2d/include", "box2d/src"])
# For the reference:
# - CCFLAGS are compilation flags shared between C and C++
# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/"])
sources = [Glob("src/*.cpp"),Glob("src/bodies/*.cpp"),Glob("src/joints/*.cpp"),Glob("src/servers/*.cpp"),Glob("src/shapes/*.cpp"),Glob("src/spaces/*.cpp"),Glob("src/box2d-wrapper/*.cpp")]
sources.extend([Glob("box2d/src/*.c")])

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
