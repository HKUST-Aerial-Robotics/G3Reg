# Installation Instructions

```shell
git clone https://github.com/HKUST-Aerial-Robotics/G3Reg
cd G3Reg
```

## Docker

You can use Docker to simplify the installation process and avoid manual dependency management.

```shell
docker build -t g3reg .
xhost +local:root
docker run -it \
    -v "$(pwd)":/root/G3Reg \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -w /root/G3Reg \
    -e DISPLAY=$DISPLAY \
    g3reg bash
```

This will start a container with all dependencies pre-installed and mount your current project directory into the container.

## Manual Installation (Without Docker)

**a. Install packages from ubuntu source.**

```shell
sudo apt install libboost-dev libyaml-cpp-dev libomp-dev libtbb-dev
```

**b. Follow the official guidance to install [GTSAM-4.1.1](https://github.com/borglab/gtsam/releases/tag/4.1.1), [PCL-1.10](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.10.0), [GLOG](https://github.com/google/glog).**

> **Note**:
>
> - `GTSAM-4.2` is not compatible with `Eigen-3.4.0`. Use a version below 3.4.0, such as `Eigen-3.3.7`.
> - `PCL-1.11` and later versions remove support for `boost::make_shared` in favor of `std::make_shared`, which is not compatible with this project. Use a version below 1.11, such as `PCL-1.10`.
> - To install the PCL visualization module, install VTK and Qt before installing PCL.

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

## Build and Compile

```shell
mkdir build && cd build
cmake ..
make -j4
```
