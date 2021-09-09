mkdir -p deps
cd deps
get eigen
curl -OL https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -zxf eigen-3.3.9.tar.gz

rm -rf libccd
git clone --depth 1 --branch v2.1 https://github.com/danfis/libccd.git

rm -rf octomap 
git clone --depth 1 --branch v1.8.0 https://github.com/OctoMap/octomap.git

rm -rf fcl
git clone --depth 1 --branch v0.6.1 https://github.com/flexible-collision-library/fcl.git

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

cd ..