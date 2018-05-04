rm -rf libccd
git clone https://github.com/danfis/libccd.git
cd libccd
git pull
git checkout 64f02f741ac94fccd0fb660a5bffcbe6d01d9939
cd ..

rm -rf octomap 
git clone https://github.com/OctoMap/octomap.git
cd octomap
git pull
git checkout b8c1d62a7a64ce0a5df278503f31d73acafa97e4 
cd ..

rm -rf fcl
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
git pull
git checkout 22f375f333beccc10c527974cef96784f0841649
cd ..

# get eigen
#curl -OL https://github.com/RLovelett/eigen/archive/3.3.4.tar.gz
#tar -zxvf 3.3.4.tar.gz
