<p align="center">
	<img width="128px" src="box2d_icon.svg"/> 
	<h1 align="center">Godot Box2D - UNMAINTAINED</h1> 
</p>

## NOTE

Currently focusing more on the [godot rapier physics](https://github.com/appsinacup/godot-rapier-physics) lib. And since this is pretty much identical in functionality with that one, but, at least at the time of writting, the rapier one has:
- Serialization thanks to rust language
- Cross platform determinism
- 2d and 3d
- API that is much closer to godot one than box2d one.
- Better safety (this one is copied after Godot Physics, and that one has a lot of raw pointers, as does this implementation)

If anyone wants to continue on this or contribute, they can make a fork or ask on discord about it.

## NOTE

Currently waiting for [box2c](https://github.com/erincatto/box2c) to be released. New issues won't be fixed until then. In meantime try [rapier2d](https://github.com/appsinacup/godot-rapier-2d) physics engine.

<p align="center">
	<a href="https://github.com/appsinacup/godot-box2d/actions/workflows/runner.yml">
        <img src="https://github.com/appsinacup/godot-box2d/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="chat on Discord"></a>
    <a href="https://github.com/erincatto/box2c" alt="Box2C Version">
        <img src="https://img.shields.io/badge/Box2C-v3.0.0-%23478cbf?logoColor=white" /></a>
    <a href="https://github.com/godotengine/godot-cpp" alt="Godot Version">
        <img src="https://img.shields.io/badge/Godot-v4.2-%23478cbf?logo=godot-engine&logoColor=white" /></a>
    <a href="https://github.com/appsinacup/godot-box2d/graphs/contributors" alt="Contributors">
        <img src="https://img.shields.io/github/contributors/appsinacup/godot-box2d" /></a>
    <a href="https://github.com/appsinacup/godot-box2d/pulse" alt="Activity">
        <img src="https://img.shields.io/github/commit-activity/m/appsinacup/godot-box2d" /></a>
    <a href="https://discord.gg/56dMud8HYn">
        <img src="https://img.shields.io/discord/1138836561102897172?logo=discord"
            alt="Chat on Discord"></a>
</p>
<p align="center">
<img src="stability-comparison.gif"/>
</p>


A [Box2D](https://github.com/erincatto/box2d) physics server for [Godot Engine](https://github.com/godotengine/godot), implemented as a GDExtension.

## Table of Contents

1. [Limitations](#limitations)
2. [Differences](#differences)
3. [Supported Platforms](#supported-platforms)
4. [Installation](#installation)
5. [Features](#features)
6. [Comparison](#comparison)
7. [License](#license)

## Limitations

- Missing circles and capsules skewing.
- Missing thread-safety.
- Missing double precision builds.
- Missing cross platform determinism.

## Differences

- Polygons have a small skin, which can result in differences from Godot Physics. [Collision shapes behave as if they are bigger than what it should be](https://github.com/appsinacup/godot-box2d/issues/72)

## Supported Platforms

Curently the Godot Box2d addon builds for:

- Windows (x86_64, x86_32)
- macOS (x86-64 + arm64 Universal)
- Linux (x86_64)
- Android (arm64, arm32, x86_64, x86_32)
- iOS (arm64)
- Web (wasm32)

## Installation

- Automatic (Recommended): Download the plugin from the official [Godot Asset Store](https://godotengine.org/asset-library/asset/2007) using the `AssetLib` tab in Godot.
- Manual: Download the [Github Release](https://github.com/appsinacup/godot-box2d/releases/latest) `godot-box2d.zip` and move only the `addons\` folder into your project `addons\` folder.

After installing, go to `Advanced Settings` -> `Physics` -> `2D`. Change `Physics Engine` to `Box2D`.

Video Tutorial:

[![Tutorial](https://img.youtube.com/vi/T_vFVh5qZiY/0.jpg)](https://www.youtube.com/watch?v=T_vFVh5qZiY)

## Features

### Improved stability

- Improved physics stability in some cases with high number of rigidbodies.

- Improves polygon collision by fixing [ghost collision](https://box2d.org/posts/2020/06/ghost-collisions/).

- Improves joints by [predictive joint limits](https://box2d.org/posts/2020/04/predictive-joint-limits/).

### Determinism

Box2D is binary deterministic. Godot Box2D should also be binary deterministic, however no such tests were run yet. The newest version of Box2D, v3, will also support cross determinism. When that is done, will also add it here.

## Comparison

Watch a comparison to Godot Physics 2D and [Rapier 2D](https://github.com/appsinacup/godot-rapier-2d) physics plugin:

[![Comparison](https://img.youtube.com/vi/wgUiZ7E19eM/0.jpg)](https://www.youtube.com/watch?v=wgUiZ7E19eM)

Or read about it on [appsinacup.com/godot-physics-vs-box2d-vs-rapier2d](https://appsinacup.com/godot-physics-vs-box2d-vs-rapier2d/)

## License

The Box2D library is developed and maintained by Erin Catto and is provided under the MIT license.

All code in this repository is provided under the MIT license. See `LICENSE` for more details and `THIRDPARTY` for third-party licenses.

Based on [rburing/physics_server_box2d](https://github.com/rburing/physics_server_box2d). Many thanks to you for starting implementation on this!
