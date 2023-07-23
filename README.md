![Box2D Logo](box2d_icon.svg)

# Godot Box2D
[![🔗 Build Status](https://github.com/godot-box2d/godot-box2d/actions/workflows/runner.yml/badge.svg)](https://github.com/godot-box2d/godot-box2d/actions/workflows/runner.yml)

A [box2D](https://github.com/erincatto/box2d) physics server for [Godot Engine](https://github.com/godotengine/godot) 4.1, implemented as a GDExtension.

Based of [rburing/physics_server_box2d](https://github.com/rburing/physics_server_box2d).

## Missing/Not implemented

- Skewed shapes
- Scaled shapes
- Constant speed on static bodies
- Collision layers and masks don't work exactly the same (having non symetric layer/mask)
- Body pickable
- Torque uses wrong values

## Install from binaries

Currently it's built automatically for:

- Windows (x86-64, x86)
- Linux (x86-64)
- macOS (x86-64 + Apple Silicon)
- iOS (arm64)
- Android (arm64 + x86_64)

Go to any action workflow on this project: [Actions List](https://github.com/rburing/physics_server_box2d/actions)

1. [Download latest release](https://github.com/godot-box2d/godot-box2d/releases/latest) from github job
2. Extract the ZIP archive and move the `addons/` folder it contains into your project folder
3. Open your project settings
4. Make sure "Advanced Settings" is enabled
5. Go to "Physics" and then "2D"
6. Change "Physics Engine" to "Box2D"
7. Restart Godot

## Building from source

1. Clone the git repository, including its `box2d` and `godot-cpp` submodules.

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
