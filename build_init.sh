#!/bin/sh

mkdir -p build.debug
cd build.debug
cmake -DCMAKE_TOOLCHAIN_FILE=../dap-sdk/cmake/Toolchain_host_Linux_target_armhf_linaro.cmake -DCMAKE_BUILD_TYPE=Debug ../
cd ..

mkdir -p build.release
cd build.release
cmake -DCMAKE_TOOLCHAIN_FILE=../dap-sdk/cmake/Toolchain_host_Linux_target_armhf_linaro.cmake ../
cd ..
