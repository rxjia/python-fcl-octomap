import os
import sys

from setuptools import Extension, setup
from Cython.Build import cythonize


def get_include_dirs():
    platform_supported = False
    for prefix in ["darwin", "linux", "bsd"]:
        if prefix in sys.platform:
            platform_supported = True
            include_dirs = [
                "/usr/include",
                "/usr/local/include",
                "/usr/include/eigen3",
                "/usr/local/include/eigen3",
            ]

            if "CPATH" in os.environ:
                include_dirs += os.environ["CPATH"].split(":")

            break
    if sys.platform == "win32":
        platform_supported = False
    if not platform_supported:
        raise NotImplementedError(sys.platform)

    # get the numpy include path from numpy
    import numpy

    include_dirs.append(numpy.get_include())
    return include_dirs


def get_libraries_dir():
    for prefix in ["darwin", "linux", "bsd"]:
        if prefix in sys.platform:
            platform_supported = True
            lib_dirs = ["/usr/lib", "/usr/local/lib"]

            if "LD_LIBRARY_PATH" in os.environ:
                lib_dirs += os.environ["LD_LIBRARY_PATH"].split(":")
            return lib_dirs
    raise NotImplementedError(sys.platform)


setup(
    ext_modules=cythonize(
        [
            Extension(
                "fcl.fcl",
                ["src/fcl/fcl.pyx"],
                include_dirs=get_include_dirs(),
                library_dirs=get_libraries_dir(),
                libraries=["fcl", "octomap"],
                language="c++",
                extra_compile_args=["-std=c++11"],
            )
        ],
    )
)
