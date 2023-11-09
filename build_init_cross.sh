#!/bin/sh

#TOOLCHAIN_FILE=../dap-sdk/share/cmake/Toolchain_host_Linux_target_armhf_linaro.cmake
TOOLCHAIN_FILE=/opt/dap-sdk/share/cmake/Toolchain_host_Linux_target_armel.cmake
#TOOLCHAIN_FILE=../dap-sdk/cmake/Toolchain_host_Buster_target_armel.cmake
CMAKE_CMD=debian_10_cmake.sh

mkdir -p build.debug
cd build.debug
rm -rf *
$CMAKE_CMD -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DCMAKE_INSTALL_PREFIX=/opt/cyclone_v_devices/adc105000_4 -DCMAKE_BUILD_TYPE=Debug ../
cd ..

mkdir -p build.release
cd build.release
rm -rf *
$CMAKE_CMD -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DCMAKE_INSTALL_PREFIX=/opt/cyclone_v_devices/adc105000_4 ../
cd ..
