
env = SConscript("godot-cpp/SConstruct")
## Libs
lib_folder = "box2d/build/bin"
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
# tweak this if you want to use different folders, or more folders, to store your source code in.
env.Append(CPPPATH=["src/"])
sources = [Glob("src/*.cpp"),Glob("src/bodies/*.cpp"),Glob("src/joints/*.cpp"),Glob("src/servers/*.cpp"),Glob("src/shapes/*.cpp"),Glob("src/spaces/*.cpp"),Glob("src/box2d-wrapper/*.cpp")]

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
