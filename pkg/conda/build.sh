#!/bin/sh

export NP_INC=`python -c "import numpy; print numpy.get_include()"`
echo "numpy include directory: " $NP_INC

export CXX="clang++"
export CC="clang"

python setup.py build_ext --include-dirs $PREFIX/include:$NP_INC --library-dirs $PREFIX/lib
python setup.py install

# TODO: cython has an issue loading .pyc compiled from another directory then where they are stored now...
# see:
find $RECIPE_DIR -name "*.pyc" -delete