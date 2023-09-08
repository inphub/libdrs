#!/bin/sh

#TOOLCHAIN_FILE=../dap-sdk/share/cmake/Toolchain_host_Linux_target_armhf_linaro.cmake
TOOLCHAIN_FILE=/opt/dap-sdk/share/cmake/Toolchain_host_Linux_target_armel.cmake
#TOOLCHAIN_FILE=../dap-sdk/cmake/Toolchain_host_Buster_target_armel.cmake
CMAKE_CMD=cmake

mkdir -p build.debug
cd build.debug
rm -rf *
$CMAKE_CMD -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE -DCMAKE_BUILD_TYPE=Debug ../
cd ..

exit 0

mkdir -p build.release
cd build.release
rm -rf *
$CMAKE_CMD -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE ../
cd ..
