# Step-by-step installation instructions
**a. Install packages from ubuntu source.**
```shell
sudo apt install libgoogle-glog-dev libboost-dev libyaml-cpp-dev libomp-dev
```
**b. Follow the official guidance to install [GTSAM-4.2](https://github.com/borglab/gtsam/tree/4f66a491ffc83cf092d0d818b11dc35135521612), [PCL](https://github.com/PointCloudLibrary/pcl).**

**c. Install iGraph 0.9.9 (To support [3DMAC](https://github.com/zhangxy0517/3D-Registration-with-Maximal-Cliques))**
```shell
sudo apt-get install flex bison
git clone https://github.com/igraph/igraph.git
cd igraph
git checkout 0.9.9
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
**d. Build**
```angular2html
cd G3Reg
mkdir build && cd build
cmake ..
make -j4
```