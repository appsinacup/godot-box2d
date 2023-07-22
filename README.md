# PhysicsServerBox2D

An unofficial [**Box2D**](https://github.com/erincatto/box2d) physics server for [**Godot Engine**](https://github.com/godotengine/godot) 4.1, implemented as a GDExtension.

The goal of the project is to be a drop-in solution for 2D physics in Godot 4.1. In your Godot project you can load the GDExtension, change the (advanced) project setting `physics/2d/physics_engine` to `Box2D`, and it will work with Godot's original 2D physics nodes such as `RigidBody2D` and `StaticBody2D`.

## Current state

⚠ This project is a work in progress. ⚠

Missing functionality:

- Pickable not implemented
- Collision layer and mask don't work same as in godot
- Torque and rotation doesn't work same as in godot (different values)
- Skewed/scaled shapes.
- Separation Ray works as a segment.
- Pin joint doesn't have softness
- Some joint properties(max force, etc.)

Things that work:

Bodies:
- [x] Rigid Body
- [] Kinematic Body
- [x] Static Body
- [x] Area

Joints:
- [x] Pin Joint
- [x] Damped Spring Joint
- [x] Groove Joint

Shapes:
- [x] Capsule Shape
- [x] Circle Shape
- [x] Concave Polygon Shape
- [x] Convex Polygon Shape
- [x] Rectangle Shape
- [x] Segment Shape
- [x] Separation Ray Shape
- [x] World Boundary Shape

Direct State:
- [x] Direct Body State
- [x] Direct Space State


## Install from build binaries

Currently it's built automatically for:

- Windows (x86-64, x86)
- Linux (x86-64)
- macOS (x86-64 + Apple Silicon)
- iOS (arm64)
- Android (arm64 + x86_64)

NOTE: the builds are not signed right now, so you might get a warning if you download for mac for eg.


Go to any action workflow on this project: [Actions List](https://github.com/rburing/physics_server_box2d/actions)

1. [Download worflow artifacts](https://docs.github.com/en/actions/managing-workflow-runs/downloading-workflow-artifacts) from github job
2. Extract the ZIP archive and move the `addons/` folder it contains into your project folder
3. Open your project settings
4. Make sure "Advanced Settings" is enabled
5. Go to "Physics" and then "2D"
6. Change "Physics Engine" to "Box2D"
7. Restart Godot

## Building from source

1. Clone the git repository https://github.com/rburing/physics_server_box2d, including its `box2d` and `godot-cpp` submodules.

2. Open a terminal application and change its working directory to the `physics_server_box2d` git repository.

3. Compile `godot-cpp` for the desired `target` (`template_debug` or `template_release`):

       cd godot-cpp
       scons target=template_debug generate_bindings=yes

4. Hack to disable b2Assert. Run:

On linux:

```
sed -i 's/#define b2Assert(A) assert(A)/#define b2Assert(A) ((void)(A))/g' ./box2d/include/box2d/b2_common.h
```

On macos:

```
sed -i '' 's/#define b2Assert(A) assert(A)/#define b2Assert(A) ((void)(A))/g' ./box2d/include/box2d/b2_common.h
```

5. Compile the GDExtension for the same `target` as above:

       cd ..
       scons target=template_debug generate_bindings=no

*Note*: The `template_debug` target can also be loaded in the Godot editor.

## Demo

The Godot project in the `demo` subdirectory is an example of how to load the GDExtension.
