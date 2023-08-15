
# Implementation Progress

Currently pass rate of [Godot-Physics-Tests](https://github.com/fabriceci/Godot-Physics-Tests):

Test Count|Pass Status|Category|Notes|Test Names|
--|--|--|--|--|
134|PASS|All|
4|VERY CLOSE|RigidBody2D CCD|There is one line that causes it to fail for another reason. https://github.com/godot-box2d/godot-box2d/issues/29.|testing Continuous Collision Detection (CCD)
1|VERY CLOSE|RigidBody2D Collision Stability|If I increase the collision boundary to 3 pixels, test passes. Could be chance that godot physics test passes.| testing if 450 rectangles can be handled before instablity
2|VERY CLOSE|RigidBody2D Sleep|* All bodies are asleep after 3.5 seconds, while in Godot after 2s. Sleep time is not configurable in box2d|*testing [contact_monitor] > The body sleep and *testing the stability with a pyramid [tolerance (2.50, 0.00)] > All body are sleep
3|VERY CLOSE|RigidBody2D Torque|Final values are very close to  Godot Physics. https://github.com/godot-box2d/godot-box2d/issues/28|* Constant torque is applied, *The impulse torque is applied to the angular velocity and *The impulse torque makes the body rotate correctly
1|FAIL|CharacterBody2D|Fail for snapping to floor. https://github.com/godot-box2d/godot-box2d/issues/11
5|FAIL|Sync to Physics|https://github.com/godot-box2d/godot-box2d/issues/7|*Sync to Physics
144|PASS + VERY CLOSE|All|
6|FAIL|All|

Performance Tests, before simulation goes below 30fps:

Shape Type|Max Bodies Box2D|Max Bodies GodotPhysics2D|
--|--|--|
Capsule|1120|2300|
Concave|640|820|
Circles|4600|2980|
Concave|1940|1920|
Rectangle|2560|2080|

NOTE: the simulation for box2d goes slower (eg. 30 fps), while for godot physics it goes completely unstable (eg. when it drops to 30 fps, even much before that).

# Building

1. Clone the git repository, including its submodules.

2. Open a terminal application and change its working directory to the `godot-box2d` git repository.

3. Compile `godot-cpp` for the desired `target` (`template_debug` or `template_release`):

       cd godot-cpp
       scons target=template_debug generate_bindings=yes

4. Compile the GDExtension for the same `target` as above:

       cd ..
       scons target=template_debug generate_bindings=no

*Note*: The `template_debug` target can also be loaded in the Godot editor.

## How to update submodule

```
git submodule sync
git submodule update
git submodule foreach git pull
```

## Lint

Run `scripts/clang-tidy.sh` in order to lint.
