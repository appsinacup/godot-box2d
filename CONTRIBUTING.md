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
git submodule update --recursive --remote
git submodule foreach git pull
```

## Lint

Run `scripts/clang-tidy.sh` in order to lint.
