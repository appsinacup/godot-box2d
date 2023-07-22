scons target=template_debug generate_bindings=no
rm -rf ./Godot-Physics-Tests/addons
cp -r dist/addons ./Godot-Physics-Tests/addons
