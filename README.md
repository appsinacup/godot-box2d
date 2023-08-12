<p align="center">
	<img width="128px" src="box2d_icon.svg"/> 
	<h1 align="center">Godot Box2D <img src="https://img.shields.io/badge/Godot-v4.1-%23478cbf?logo=godot-engine&logoColor=white"/><img src="https://img.shields.io/badge/Box2D-v2.4.1-%23478cbf?logoColor=white"/></h1> 
</p>
<p align="center">
<img src="stability-comparison.gif"/>
</p>

[![🔗 Build Status](https://github.com/appsinacup/godot-box2d/actions/workflows/runner.yml/badge.svg)](https://github.com/godot-box2d/godot-box2d/actions/workflows/runner.yml)

A [Box2D](https://github.com/erincatto/box2d) physics server for [Godot Engine](https://github.com/godotengine/godot), implemented as a GDExtension.

# Installation

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2007) using the `AssetLib` tab in Godot.
- Manual: Download the source code and move only the addons folder into your project addons folder.

After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Box2D`.

# Video Tutorial

[![Tutorial](https://img.youtube.com/vi/T_vFVh5qZiY/0.jpg)](https://www.youtube.com/watch?v=T_vFVh5qZiY)

# Features

- Improved physics stability in some cases with high number of rigidbodies.

- Improves polygon collision by fixing [ghost collision](https://box2d.org/posts/2020/06/ghost-collisions/).

- Improves joints by [predictive joint limits](https://box2d.org/posts/2020/04/predictive-joint-limits/).

- Box2D is binary deterministic. Godot Box2D should also be binary deterministic, however no such tests were run yet.


# Limitations

- Having non symetrical physics mask/layers results in collision on both bodies.
- Circles and capsules only support uniform scaling and don't support skewing.
- Mass works as density instead of total mass.

# Roadmap

- Cross Platform Determinism
- Add more types of joints
- Pass all Godot Physics Tests.

# [Discord](https://discord.gg/56dMud8HYn)

A vibrant community for discussion, user support and showcases.

# License

The Box2D library is developed and maintained by Erin Catto and is provided under the MIT license.

All code in this repository is provided under the MIT license. See `LICENSE` for more details and `THIRDPARTY` for third-party licenses.

Based on [rburing/physics_server_box2d](https://github.com/rburing/physics_server_box2d). Many thanks to you for starting implementation on this!
