#!/bin/bash

NS3_VERSION=3.35

if [ $# -ne 0 -a $# -ne 1 ]; then
    echo "Too many arguments. You shall specify:"
    echo "	(Optional) 'install-dependencies' to attempt the installation of the ns-${NS3_VERSION} main dependencies"

    exit 1
fi

if [ "$EUID" -eq 0 ]; then
	echo "Please do NOT run this script as root."
	exit 1
fi

if [ -d "./ns-3-allinone" ]; then
    echo "Error: cannot proceed if there is a directory named ns-3-allinone is the current path."
    echo "Please rename it or move/launch the script from another directory."

    exit 1
fi

read -p "Press ENTER to continue with the installation..."

if [ ! -z $1 ]; then
	if [ $1 == "install-dependencies" ]; then
		echo "Installing some of the main dependencies for ns-${NS3_VERSION}..."
		sudo apt update
		if [ $? -ne 0 ]; then
			echo "Error: cannot run 'apt update'."
			exit 1
		fi

		sudo apt install -y g++ python3 cmake ninja-build git ccache gir1.2-goocanvas-2.0 python3-gi python3-gi-cairo python3-pygraphviz gir1.2-gtk-3.0 python3-dev pkg-config sqlite3 cmake python3-setuptools qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools mercurial unzip gdb valgrind clang clang-format doxygen graphviz imagemagick python3-sphinx gsl-bin libgsl-dev libgslcblas0 tcpdump libsqlite3-dev libxml2 libxml2-dev libc6-dev libc6-dev-i386 libclang-dev llvm-dev automake python3-pip libgtk-3-dev libxml2 libxml2-dev libboost-all-dev libgrpc++-dev libprotobuf-dev protobuf-compiler libdpdk-dev libssh-dev
		sudo apt install build-essential autoconf libtool pkg-config cmake
		python3 -m pip install --user grpcio
		python3 -m pip install --user grpcio-tools
		python3 -m pip install --user conan==1.54.0	

		echo "Installing gRPC from source..."
		git clone -b v1.60.0 https://github.com/grpc/grpc
		cd grpc
 		git submodule update --init

		mkdir -p cmake/build
		cd cmake/build
		cmake ../..
		make -j$(nproc)
		sudo make install
		cd ../../..

		echo "Installing OpenCV..."
		git clone https://github.com/opencv/opencv.git
	    git clone https://github.com/opencv/opencv_contrib.git
	    cd opencv
	    mkdir build
	    cd build
	    cmake -D CMAKE_BUILD_TYPE=Release \
	          -D CMAKE_INSTALL_PREFIX=/usr/local \
	          -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
	          -D WITH_TBB=ON \
	          -D WITH_V4L=ON \
	          -D WITH_QT=OFF \
	          -D WITH_OPENGL=ON ..
	    make -j$(nproc)
	    sudo make install
	    ldconfig
	    cd ../..

		# Detecting the current Ubuntu version to install the correct version of libgsl
		# This is done only on Ubuntu (i.e. only if the command "lsb_release" returns "Ubuntu" as distro)
		# If Ubuntu >= 20.10 is detected, libgsl25 is installed, otherwise libgsl23 is installed
		# If the distro is not detected to be Ubuntu, for the time being, the script attempts to install libgsl23
		# Please report any bug when using "install-dependencies": we will try to fix it as soon as possible!
		lsb_release -d | grep Ubuntu > /dev/null
		
		if [ $? -eq 0 ]; then
			version=$(lsb_release -d | cut -d " " -f2 | cut -d "." -f1)
			subversion=$(lsb_release -d | cut -d " " -f2 | cut -d "." -f2)
			
			if [ $version -gt 20 -a $version -lt 22 -o \( $version -ge 20 -a $subversion -ge 10 \) ]; then
				echo "Detected Ubuntu >= 20.10 and Ubuntu < 22.04 - installing libgsl25"
				sudo apt install libgsl25
			elif [ $version -gt 22 -o \( $version -ge 22 -a $subversion -ge 04 \) ]; then
				echo "Detected Ubuntu >= 22.04 - installing libgsl27"
				sudo apt install libgsl27
			else
				echo "Detected Ubuntu < 20.10 - installing libgsl23"
				sudo apt install libgsl23
			fi
		else
			echo "Detected a distribution different than Ubuntu - installing libgsl23"
			sudo apt install libgsl23
		fi

		if [ $? -ne 0 ]; then
			echo "Error: cannot install the dependencies."
			exit 1
		fi
	else
		echo "Error: the second optional argument can be equal to \"install-dependencies\" or left empty only."
		echo "\"$1\" is not a valid optional argument."
		exit 1
	fi
fi

echo "Downloading ns-${NS3_VERSION} from the official repository..."
sleep 1
set -v
git clone https://gitlab.com/cttc-lena/ns-3-dev.git
cd ns-3-dev/src/
git clone https://gitlab.com/cttc-lena/nr.git
cd nr
git checkout nr-v2x-dev
find . -type d -name .git -exec rm -rfv {} \;
find . -type f -name .gitignore -exec rm -rfv {} \;
find . -type f -name .gitlab-ci-clang.yml -exec rm -rfv {} \;
find . -type f -name .gitlab-ci-gcc.yml -exec rm -rfv {} \;
find . -type f -name .gitlab-ci.yml -exec rm -rfv {} \;
find . -type f -name .gitmodules -exec rm -rfv {} \;
cd ../..
git checkout ns-3-dev-v2x-v0.2
set +v

echo "Removing git from vanilla ns-${NS3_VERSION}..."
sleep 1
set -v
find . -type d -name .git -exec rm -rfv {} \;
find . -type f -name .gitignore -exec rm -rfv {} \;
find . -type f -name .gitattributes -exec rm -rfv {} \;
set +v

echo "Installing the sandbox..."
sleep 1
set -v
cd ..
shopt -s extglob
shopt -s dotglob
cp -af ./!(ns-3-dev) ns-3-dev/
set +v

echo "Patching CMakeLists.txt to solve compatibility issue with C source files"
sleep 1
set -v
cd ns-3-dev
sed -i -E 's#^([[:blank:]]*)project\(NS3 CXX\)#\1project\(NS3 C CXX\)#' CMakeLists.txt
cd ..
set +v

echo "Moving the full installation to the current directory..."
sleep 1
set -v
rm -rfv AUTHORS .git .gitignore img LICENSE license_gplv2.txt README.md src switch_ETSI_version.sh VERSION enable_v2x_emulator.sh
set +v

echo "Extending available path loss models..."
sleep 1
set -v
cd ns-3-dev
cp src/automotive/propagation-extended/cni-urbanmicrocell-propagation-loss-model.cc src/propagation/model/
cp src/automotive/propagation-extended/cni-urbanmicrocell-propagation-loss-model.h src/propagation/model/
cp src/automotive/propagation-extended/CMakeLists.txt src/propagation/

echo "Copying propagation files for NVIDIA Sionna..."
sleep 1
cp src/sionna/files/propagation/CMakeLists.txt src/propagation/
cp src/sionna/files/propagation/propagation-delay-model.cc src/propagation/model/
cp src/sionna/files/propagation/propagation-delay-model.h src/propagation/model/
cp src/sionna/files/propagation/propagation-loss-model.cc src/propagation/model/
cp src/sionna/files/propagation/propagation-loss-model.h src/propagation/model/
cp src/sionna/files/propagation/three-gpp-propagation-loss-model.h src/propagation/model
cp src/sionna/files/propagation/three-gpp-propagation-loss-model.cc src/propagation/model
cp src/sionna/files/spectrum/CMakeLists.txt src/spectrum/
cp src/sionna/files/spectrum/three-gpp-spectrum-propagation-loss-model.cc src/spectrum/model

cp src/automotive/model/TxTracker/channel_files/modified/yans-wifi-phy.h src/wifi/model

echo "Extending Signal Info features..."
sleep 1
cp src/automotive/model/SignalInfo/rssi-tag.cc src/wifi/model/
cp src/automotive/model/SignalInfo/rssi-tag.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/rssi-tag.cc src/nr/model/
cp src/automotive/model/SignalInfo/rssi-tag.cc src/lte/model/
cp src/automotive/model/SignalInfo/rssi-tag.h src/wifi/model/
cp src/automotive/model/SignalInfo/rssi-tag.h src/cv2x/model/
cp src/automotive/model/SignalInfo/rssi-tag.h src/nr/model/
cp src/automotive/model/SignalInfo/rssi-tag.h src/lte/model/
cp src/automotive/model/SignalInfo/timestamp-tag.cc src/wifi/model/
cp src/automotive/model/SignalInfo/timestamp-tag.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/timestamp-tag.cc src/nr/model/
cp src/automotive/model/SignalInfo/timestamp-tag.cc src/lte/model/
cp src/automotive/model/SignalInfo/timestamp-tag.h src/wifi/model/
cp src/automotive/model/SignalInfo/timestamp-tag.h src/cv2x/model/
cp src/automotive/model/SignalInfo/timestamp-tag.h src/nr/model/
cp src/automotive/model/SignalInfo/timestamp-tag.h src/lte/model/
cp src/automotive/model/SignalInfo/rsrp-tag.cc src/wifi/model/
cp src/automotive/model/SignalInfo/rsrp-tag.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/rsrp-tag.cc src/nr/model/
cp src/automotive/model/SignalInfo/rsrp-tag.cc src/lte/model/
cp src/automotive/model/SignalInfo/rsrp-tag.h src/wifi/model/
cp src/automotive/model/SignalInfo/rsrp-tag.h src/cv2x/model/
cp src/automotive/model/SignalInfo/rsrp-tag.h src/nr/model/
cp src/automotive/model/SignalInfo/rsrp-tag.h src/lte/model/
cp src/automotive/model/SignalInfo/sinr-tag.cc src/wifi/model/
cp src/automotive/model/SignalInfo/sinr-tag.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/sinr-tag.cc src/nr/model/
cp src/automotive/model/SignalInfo/sinr-tag.cc src/lte/model/
cp src/automotive/model/SignalInfo/sinr-tag.h src/wifi/model/
cp src/automotive/model/SignalInfo/sinr-tag.h src/cv2x/model/
cp src/automotive/model/SignalInfo/sinr-tag.h src/nr/model/
cp src/automotive/model/SignalInfo/sinr-tag.h src/lte/model/
cp src/automotive/model/SignalInfo/size-tag.cc src/wifi/model/
cp src/automotive/model/SignalInfo/size-tag.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/size-tag.cc src/nr/model/
cp src/automotive/model/SignalInfo/size-tag.cc src/lte/model/
cp src/automotive/model/SignalInfo/size-tag.h src/wifi/model/
cp src/automotive/model/SignalInfo/size-tag.h src/cv2x/model/
cp src/automotive/model/SignalInfo/size-tag.h src/nr/model/
cp src/automotive/model/SignalInfo/size-tag.h src/lte/model/

cp src/automotive/model/SignalInfo/WiFi/wifi-mac-queue-item.h src/wifi/model/
cp src/automotive/model/SignalInfo/WiFi/ocb-wifi-mac.cc src/wave/model/
cp src/automotive/model/SignalInfo/WiFi/frame-exchange-manager.cc src/wifi/model/
cp src/automotive/model/SignalInfo/WiFi/qos-frame-exchange-manager.cc src/wifi/model/
cp src/automotive/model/SignalInfo/WiFi/CMakeLists.txt src/wifi/

cp src/automotive/model/SignalInfo/CV2X/cv2x_lte-spectrum-phy.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/CV2X/cv2x_lte-spectrum-phy.h src/cv2x/model/
cp src/automotive/model/SignalInfo/CV2X/cv2x_lte-ue-mac.h src/cv2x/model/
cp src/automotive/model/SignalInfo/CV2X/cv2x_lte-ue-mac.cc src/cv2x/model/
cp src/automotive/model/SignalInfo/CV2X/CMakeLists.txt src/cv2x/

cp src/automotive/model/SignalInfo/NR/nr-spectrum-phy.cc src/nr/model/
cp src/automotive/model/SignalInfo/NR/nr-spectrum-phy.h src/nr/model/
cp src/automotive/model/SignalInfo/NR/nr-ue-phy.cc src/nr/model/
cp src/automotive/model/SignalInfo/NR/CMakeLists.txt src/nr/

cp src/automotive/model/SignalInfo/LTE/lte-spectrum-phy.cc src/lte/model/
cp src/automotive/model/SignalInfo/LTE/lte-ue-phy.cc src/lte/model/
cp src/automotive/model/SignalInfo/LTE/lte-ue-phy.h src/lte/model/
cp src/automotive/model/SignalInfo/LTE/CMakeLists.txt src/lte/

if ! command -v lsb_release >/dev/null 2>&1; then
    echo "WARNING: lsb_release not installed. Try to install to detect Ubuntu's version."
    if [ -x "$(command -v apt-get)" ]; then
        sudo apt-get update && sudo apt-get install -y lsb-release
    else
        echo "Packet manager not found. Please install 'lsb-release' manually."
        exit 1
    fi
fi

VERSION=$(lsb_release -rs)
if [ -z "$VERSION" ]; then
    echo "ERROR: Impossible to identify Ubuntu version."
    exit 1
fi
if [ "$VERSION" == "24.04" ]; then
    echo "You are using Ubuntu 24.04 (Noble Numbat)."
    find src/automotive -type f -name "asn_system.h" -exec sed -i 's/#define SIZE_MAX.*/#define SIZE_MAX __SIZE_MAX__/g' {} +
    find src -type f -not -path "*/proto/*" -exec sed -i 's/\bTYPE_DOUBLE\b/TRACI_TYPE_DOUBLE/g' {} +
    find src -type f -not -path "*/proto/*" -exec sed -i 's/\bTYPE_STRING\b/TRACI_TYPE_STRING/g' {} +
    find src -type f -not -path "*/proto/*" -exec sed -i 's/\bTYPE_FLOAT\b/TRACI_TYPE_FLOAT/g' {} +
    find src -type f -not -path "*/proto/*" -exec sed -i 's/\bTYPE_BOOL\b/TRACI_TYPE_BOOL/g' {} +
	sed -i 's/#include <vector>/#include <vector>\n#include <cstdint>/' ./src/wifi/model/block-ack-type.h
	sed -i 's/#include <vector>/#include <vector>\n#include <cstdint>/' ./src/network/utils/bit-serializer.h
	sed -i 's/#include <vector>/#include <vector>\n#include <cstdint>/' ./src/network/utils/bit-deserializer.h
elif [ "$VERSION" == "22.04" ]; then
    echo "You are using Ubuntu 22.04 (Jammy Jellyfish)."
elif [ "$VERSION" == "20.04" ]; then
    echo "You are using Ubuntu 20.04 (Focal Fossa)."
else
    echo "ERROR: the version $VERSION is not supported by VaN3Twin, please use this tool with Ubuntu 20.04, Ubuntu 22.04, or Ubuntu 24.04"
fi

cd ..
set +v

echo "Installation completed. You will find a copy of this script in ./ns-3-dev."

rm $0
