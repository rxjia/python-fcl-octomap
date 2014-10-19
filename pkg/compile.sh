#!/bin/bash
export PREFIX=~/miniconda/
export NUMPY_INC=/Users/jelleferinga/miniconda/pkgs/numpy-1.8.2-py27_0/lib/python2.7/site-packages/numpy/core/include
export FCL_PY_DIR=/Users/jelleferinga/miniconda/lib/python2.7/site-packages/fcl

export HERE=`pwd`

# UGH: fcl/fcl.cpp:14950:3: warning: conversion from string literal to 'char *' is deprecated
#      [-Wdeprecated-writable-strings]

rm ../fcl/*.cpp
rm -rf ../build/*

cd ../

echo "compiling cython FCL wrapper"
#python setup.py build_ext # --include-dirs $PREFIX/include --library-dirs $PREFIX/lib
python setup.py install
echo "done compiling "

echo "run install name"
install_name_tool -change libfcl.dylib @loader_path/../../../libfcl.dylib $FCL_PY_DIR/fcl.so
install_name_tool -change libfcl.dylib @loader_path/../../../libfcl.dylib $FCL_PY_DIR/mesh.so

# mehhh, keeps on failing on the already compiled files... strange...
rm $FCL_PY_DIR/*.pyc

# conflicts with the `fcl` project directory
# looks for the fcl.so module here and not finding it
cd ~
#echo "running fcl unit test"
#nosetests ~/GIT/python-fcl/test/fcl_unit_test.py
echo "running mesh unit test"
nosetests -s ~/GIT/python-fcl/test/mesh_unit_test.py
#echo "done..."
cd $HERE