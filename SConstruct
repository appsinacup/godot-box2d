#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

## Libs
if env["dev_build"]:
    lib_folder = "box2d/build/"
else:
    lib_folder = "box2d/build/release"

if env["platform"] == "ios":
	env.Append(LINKFLAGS='-framework Security')

if env["platform"] == "windows":
	lib_file = "box2d{}"
	lib = lib_file.format(env["LIBSUFFIX"])
else:
	lib_file = "libbox2d{}"
	lib = lib_file.format(env["LIBSUFFIX"])
env.Append(LIBPATH=[lib_folder])
env.Append(LIBS=[lib])

env.Prepend(CPPPATH=["box2d/include"])

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
sources = [Glob("src/*.cpp"),Glob("src/bodies/*.cpp"),Glob("src/joints/*.cpp"),Glob("src/servers/*.cpp"),Glob("src/shapes/*.cpp"),Glob("src/spaces/*.cpp"),Glob("src/box2d-wrapper/*.cpp")]

if env["platform"] == "windows":
    env.Append(CPPDEFINES="WINDOWS_ENABLED")

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
