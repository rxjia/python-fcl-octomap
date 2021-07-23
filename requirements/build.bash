echo "Install eigen"
cmake -B build -S eigen-3.3.9
cmake --install build

echo "Build and install libccd"
cd libccd
cmake .
make -j4
make install
cd ..

echo "Build and install octomap"
cd octomap 
cmake .
make -j4
make install
cd ..

echo "Build and install fcl"
cd fcl
cmake .
make -j4
make install
cd ..
