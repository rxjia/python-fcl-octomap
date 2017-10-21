# NOTE: this docker file is 

FROM quay.io/pypa/manylinux1_x86_64:latest

RUN yum install -y gcc 

COPY requirements/install_cmake.bash .
RUN bash install_cmake.bash

COPY requirements/install_pip.bash .
RUN bash install_pip.bash

COPY requirements/clone.bash .
RUN bash clone.bash

COPY requirements/build.bash .
RUN bash build.bash

COPY . /python_fcl
#RUN pip install cibuildwheel==0.6.0
#RUN cd /python_fcl && cibuildwheel --output-dir wheelhouse
