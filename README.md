<p align="center">
	<img width="128px" src="box2d_icon.svg"/> 
	<h1 align="center">Godot Box2D</h1> 
</p>

<p align="center">
	<a href="https://github.com/appsinacup/godot-box2d/actions/workflows/runner.yml">
        <img src="https://github.com/appsinacup/godot-box2d/actions/workflows/runner.yml/badge.svg?branch=main"
            alt="chat on Discord"></a>
    <a href="https://github.com/erincatto/box2d" alt="Box2D Version">
        <img src="https://img.shields.io/badge/Box2D-v2.4.1-%23478cbf?logoColor=white" /></a>
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
2. [Supported Platforms](#supported-platforms)
3. [Installation](#installation)
4. [Features](#features)
5. [License](#license)

## Limitations

- Character Controller is not very precise. (WIP)
- Body Pickable option is not implemented. (WIP)
- Having non symetrical physics mask/layers results in collision on both bodies. (Can't fix)
- Circles and capsules only support uniform scaling and don't support skewing. (WIP? if people need this)
- Missing thread-safety. (WIP)
- Missing double precision builds. (WIP)
- Missing cross platform determinism. (WIP)
- Missing webgl builds. (WIP)

## Supported Platforms

Curently the Godot Box2d addon builds for:

- Windows (x86_32, x86_64)
- macOS (x86-64 + Apple Silicon)
- iOS (arm64)
- Linux (x86_64)
- Android (arm64, x86_64)

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

## License

The Box2D library is developed and maintained by Erin Catto and is provided under the MIT license.

All code in this repository is provided under the MIT license. See `LICENSE` for more details and `THIRDPARTY` for third-party licenses.

Based on [rburing/physics_server_box2d](https://github.com/rburing/physics_server_box2d). Many thanks to you for starting implementation on this!
