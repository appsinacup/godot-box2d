scons target=template_debug generate_bindings=no arch=arm64 dev_build=yes
rm -rf Godot-Physics-Tests/addons
cp -rf bin/addons Godot-Physics-Tests/addons

