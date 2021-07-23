rm -rf libccd
git clone --depth 1 --branch v2.1 https://github.com/danfis/libccd.git

rm -rf octomap 
git clone --depth 1 --branch v1.8.0 https://github.com/OctoMap/octomap.git

rm -rf fcl
git clone --depth 1 --branch v0.6.1 https://github.com/flexible-collision-library/fcl.git

# get eigen
curl -OL https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -zxf eigen-3.3.9.tar.gz