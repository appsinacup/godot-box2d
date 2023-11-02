cd box2d
rm -rf build
mkdir build
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DBOX2D_BUILD_SAMPLES=ON
cmake --build build --config Debug
./test/test
