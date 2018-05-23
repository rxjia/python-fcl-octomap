cd libccd
cmake .
make -j4
make install
cd ..

cd octomap 
cmake .
make -j4
make install
cd ..

cd fcl
cmake .
make -j4
make install
cd ..
