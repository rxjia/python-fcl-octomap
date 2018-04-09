# install cmake using their shell script
# we do this because fcl needs cmake >= 2.8.12, but centos 5
# from the manylinux image has only cmake 2.8.11
curl -OL https://cmake.org/files/v3.11/cmake-3.11.0-Linux-x86_64.sh
bash cmake-3.11.0-Linux-x86_64.sh --skip-license --prefix=/usr
