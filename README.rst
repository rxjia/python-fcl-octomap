python-fcl
=========

About
-----
This library is a Python binding of FCL library.

Build
-----
Building python-fcl requires FCL headers and libraries.
When building, you can specify their location with the --include-dirs
and --library-dirs command line options:

    $ python setup.py build_ext --include-dirs /path/to/includes \
                                --library-dirs /path/to/libraries

Install
-------
You can install python-fcl using the normal distutils install command:

    $ python setup.py install
