# install cmake using their shell script
# we do this because fcl needs cmake >= 2.8.12, but centos 5
# from the manylinux image has only cmake 2.8.11
curl -OL https://cmake.org/files/v2.8/cmake-2.8.12.2-Linux-i386.sh
bash cmake-2.8.12.2-Linux-i386.sh --skip-license --prefix=/usr
