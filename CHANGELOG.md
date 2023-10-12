# Changelog

## [v0.8](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.7)

- Fixes [SegmentShape2D seems unsupported](https://github.com/appsinacup/godot-box2d/issues/47).
- Fixes [Shapecast2D node does not detect collisions](https://github.com/appsinacup/godot-box2d/issues/48).

## [v0.7](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.7)

- [Zero friction physics material does not simulate](https://github.com/appsinacup/godot-box2d/issues/40)
- [Applying impulse ignores rigidbody mass and introduces a ton of angular motion.](https://github.com/appsinacup/godot-box2d/issues/41)
- Revert mass is density change. Fixes [Constant Torque and Impulse Torque are applied differently](https://github.com/appsinacup/godot-box2d/issues/28)

## [v0.6](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.6)

- Fix Polygon Skin not letting objects fall through small gaps. Also fix inside out polygons bug.
- Fix [Fix One way Collision for CharacterBody](https://github.com/appsinacup/godot-box2d/issues/33)

## [v0.5.4](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.5.4)

- Fix for [Read project global configurations](https://github.com/appsinacup/godot-box2d/issues/26)
- Fix for [Fix One Way Collision for rigidbodies](https://github.com/appsinacup/godot-box2d/issues/27)

## [v0.5.3](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.5.3)

- Fix for [godot-box2d/issues/22](https://github.com/godot-box2d/godot-box2d/issues/22)
- Fix unsafe and safe fraction bounding(between 0 and 1)
- Implement base one way direction collision(not finished).

## [v0.5.2](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.5.2)

- Fixes crashes when joints aren't set properly.
- Fixes other body test functions crashes.
- Fixes body_test_move to work correctly and not check for area collision.

## [v0.5.1](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.5.1)

- Possible fix for [collision between static body and area bug](https://github.com/godot-box2d/godot-box2d/issues/19)

## [v0.5](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.5)

- Fix [shapes not updating(changing shape geometry)](https://github.com/godot-box2d/godot-box2d/issues/16)
- Fix [deleting shapes/body/spaces bug](https://github.com/godot-box2d/godot-box2d/issues/18)

## [v0.4](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.4)

- Fix torque (almost, there is still a little difference in test, but numbers are much more closer now)
- Fix concave polygon
- Add support for conveyer belt
- Add build for web but not enabled yet (bug in godot-cpp)
- Update/fix support for skewing/scaling for shapes.

## [v0.3](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.3)

- Fix mac binary signing
- Add basic implementation for force_integration_callback

## [v0.2](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.2)

- Fix CharacterController2D to correctly work(passing 19/20 tests)
- Fix normal issue on test_motion function

## [v0.1](https://github.com/godot-box2d/godot-box2d/releases/tag/v0.1)

- Add all api's from physics server

Known issues:
- KinematicBody2D has a lot of issues still.
