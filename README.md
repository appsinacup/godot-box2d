![Box2D Logo](box2d_icon.svg)

# Godot Box2D
[![🔗 Build Status](https://github.com/godot-box2d/godot-box2d/actions/workflows/runner.yml/badge.svg)](https://github.com/godot-box2d/godot-box2d/actions/workflows/runner.yml)


A [box2D](https://github.com/erincatto/box2d) physics server for [Godot Engine](https://github.com/godotengine/godot) 4.1, implemented as a GDExtension.

## Limitations

- Having non symetrical physics mask/layers results in no collision. Eg. if we had the following:

|object|mask|category|
|---|---|---|
|object A|1|2|
|object B|2|2|

       In godot would result a collision, but would be very weird, as only one of the objects would receive collision restution.

## Missing/Not implemented

- Skewed shapes
- Scaled shapes
- Constant speed on static bodies
- Body pickable
- Torque uses wrong values

## Install

### From Godot Asset Store

[Godot Asset Library](https://godotengine.org/asset-library/asset/2007)

### From Github Releases

1. Download from [Github Releases](https://github.com/godot-box2d/godot-box2d/releases/latest)
2. Extract the ZIP archive and move the `addons/` folder it contains into your project folder
3. Open your project settings
4. Make sure "Advanced Settings" is enabled
5. Go to "Physics" and then "2D"
6. Change "Physics Engine" to "Box2D"
7. Restart Godot

## Building from source

1. Clone the git repository, including its submodules.

2. Open a terminal application and change its working directory to the `godot-box2d` git repository.

3. Compile `godot-cpp` for the desired `target` (`template_debug` or `template_release`):

       cd godot-cpp
       scons target=template_debug generate_bindings=yes

4. Compile the GDExtension for the same `target` as above:

       cd ..
       scons target=template_debug generate_bindings=no

*Note*: The `template_debug` target can also be loaded in the Godot editor.

## Lint

Run `scripts/clang-tidy.sh` in order to lint.

## Credits

Based of [rburing/physics_server_box2d](https://github.com/rburing/physics_server_box2d). Many thanks to you for starting implementation on this!
