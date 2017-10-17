# change directory to script location
cd $(dirname $(realpath $0))

cd libccd/src
make -j4
sudo make install

cd ../..

cd fcl
cmake .
make -j4
sudo make install
