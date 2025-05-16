#!/usr/bin/env sh
set -e

echo "\n\n Installing apt dependencies...................................."
    apt-get update
    apt-get install -y \
        wget \
        libegl1 \
        libgl1 \
        libgl1-mesa-dri \
        mesa-utils \
        python3-kconfiglib

echo "\n\n update submodules.............................................."
  git submodule update --init --recursive

echo "\n\n Installing PX4 Autopilot......................................."
  bash /workspace/src/PX4-Autopilot/Tools/setup/ubuntu.sh

echo "\n\n Installing Micro-XRCE-DDS-Agent................................"
    mkdir /workspace/src/Micro-XRCE-DDS-Agent/build
    cd /workspace/src/Micro-XRCE-DDS-Agent/build
    cmake ..
    make
    sudo make install
    ldconfig /usr/local/lib/

echo "\n\n cleanup........................................................"
  rm -rf /var/lib/apt/lists/*