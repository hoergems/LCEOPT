#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Install dependencies
sudo apt update
sudo apt install -y build-essential cmake libtinyxml-dev libeigen3-dev libassimp-dev libfcl-dev libboost-system-dev libboost-thread-dev libboost-filesystem-dev libsdformat-dev liburdfdom-dev

# Build and install Pinocchio
cd $SCRIPT_DIR/thirdparty/Pinocchio
mkdir -p build && cd build
cmake -DBUILD_BENCHMARK=OFF -DBUILD_UTILS=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF -DBUILD_WITH_URDF_SUPPORT=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

echo "Installed dependencies"
