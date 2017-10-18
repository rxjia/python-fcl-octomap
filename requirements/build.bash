cd libccd/src
make -j4
make install

cd ../..

cd fcl
# try using cmake by location
if [ -e /usr/bin/cmake3 ]; then
    /usr/bin/cmake3 .
elif [ -e /usr/bin/cmake28 ]; then
    /usr/bin/cmake28 .
else
    cmake .
fi
make -j4
make install
