FROM ubuntu:20.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-dev \
    libyaml-cpp-dev \
    libomp-dev \
    flex \
    bison \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install CMake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.25.3/cmake-3.25.3-linux-x86_64.sh && \
    sh cmake-3.25.3-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.25.3-linux-x86_64.sh

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    libeigen3-dev \
    pkg-config \
    libflann-dev \
    libusb-1.0-0-dev \
    libpng-dev \
    libqhull-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    zlib1g-dev \
    libpcap-dev \
    libtbb-dev

# Install GTSAM
RUN wget https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip -O /tmp/gtsam-4.1.1.zip && \
    cd /tmp && \
    unzip gtsam-4.1.1.zip && \
    cd gtsam-4.1.1 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    rm -rf /tmp/gtsam-4.1.1 /tmp/gtsam-4.1.1.zip

# Install VTK and Qt
RUN apt-get update && apt-get install -y \
    libvtk7-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libglew-dev \
    libxi-dev \
    libxmu-dev

# Install PCL 1.10.0
RUN git clone https://github.com/PointCloudLibrary/pcl.git /pcl && \
    cd /pcl && \
    git checkout pcl-1.10.0 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# Install GLOG
RUN git clone https://github.com/google/glog.git /glog && \
    cd /glog && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# Install iGraph
RUN git clone https://github.com/igraph/igraph.git /igraph && \
    cd /igraph && \
    git checkout 0.9.9 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# Set the working directory
WORKDIR /G3Reg