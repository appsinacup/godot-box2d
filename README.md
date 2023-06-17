# PhysicsServerBox2D

An unofficial [**Box2D**](https://github.com/erincatto/box2d) physics server for [**Godot Engine**](https://github.com/godotengine/godot) 4.0, implemented as a GDExtension.

The goal of the project is to be a drop-in solution for 2D physics in Godot 4.0. In your Godot project you can load the GDExtension, change the (advanced) project setting `physics/2d/physics_engine` to `Box2D`, and it will work with Godot's original 2D physics nodes such as `RigidBody2D` and `StaticBody2D`.

## Current state

⚠ This project is a work in progress, still in a very early stage. ⚠

Runtime errors of the form `Required virtual method ... must be overridden before calling` reflect this unfinished state, and they hint at which functionality is still missing.

## Building from source

1. Clone the git repository https://github.com/rburing/physics_server_box2d, including its `box2d` and `godot-cpp` submodules.

2. Open a terminal application and change its working directory to the `physics_server_box2d` git repository.

3. Compile `godot-cpp` for the desired `target` (`template_debug` or `template_release`):

       cd godot-cpp
       scons target=template_debug generate_bindings=yes

4. Compile the GDExtension for the same `target` as above:

       cd ..
       scons target=template_debug generate_bindings=no

*Note*: The `template_debug` target can also be loaded in the Godot editor.

## Demo

The Godot project in the `demo` subdirectory is an example of how to load the GDExtension.
