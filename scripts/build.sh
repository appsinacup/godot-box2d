scons target=template_release generate_bindings=no arch=arm64
rm -rf Godot-Physics-Tests/addons
cp -rf bin/addons Godot-Physics-Tests/addons

