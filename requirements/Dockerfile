# NOTE: this docker file is only for local testing of the build
# it should be copied up to the repo root and ran there
# or symlinked

FROM quay.io/pypa/manylinux1_x86_64:latest


RUN yum install -y gcc 

COPY requirements/install_cmake.bash .
RUN bash install_cmake.bash

COPY requirements/clone.bash .
RUN bash clone.bash

COPY requirements/build.bash .
RUN bash build.bash

ENV PATH=/opt/python/cp27-cp27m/bin:$PATH
RUN pip install numpy

COPY . /python_fcl
RUN cd /python_fcl && python setup.py build_ext
