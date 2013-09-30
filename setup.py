
from distutils.core import Extension, setup
from Cython.Distutils import build_ext

setup(
    name="fcl",
    version="0.1",
    packages=["fcl"],
    ext_modules=[Extension(
        "fcl",
        ["fcl/fcl.pyx"],
        libraries=[
                "fcl"
                ],
        language="c++")],
    cmdclass={'build_ext': build_ext},
    )
