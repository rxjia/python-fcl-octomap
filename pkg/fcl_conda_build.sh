#!/bin/bash
conda-build $ROBOCODE/conda/python-fcl
conda install /Users/jelleferinga/miniconda/conda-bld/osx-64/python-fcl-0.3.1-np19py27_1.tar.bz2

export FCL_PY_DIR=/Users/jelleferinga/miniconda/lib/python2.7/site-packages/fcl
rm $FCL_PY_DIR/*.pyc

nosetests ../test/fcl_unit_test.py
