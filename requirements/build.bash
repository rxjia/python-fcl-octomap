# change directory to script location
cd $(dirname $(realpath $0))

cd libccd/src
make -j4
make install

cd ../..

cd fcl

# check if cmake is called cmake3 *sigh*
if hash cmake3 2>/dev/null; then
    cmake3 .
else
    cmake .
fi

make -j4
make install
