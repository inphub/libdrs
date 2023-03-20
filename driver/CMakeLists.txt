cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project (cyclone_v_driver VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Module info
add_definitions(-D__KERNEL__ -DMODULE)

# Find the kernel release
execute_process(
        COMMAND uname -r
        OUTPUT_VARIABLE KERNEL_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set (KERNELHEADERS_DIR /opt/linux-socfpga)

message(STATUS "Kernel headers: ${KERNELHEADERS_DIR}")

# Add header files
include_directories(
        ${KERNELHEADERS_DIR}/include
        ${KERNELHEADERS_DIR}/arch/arm/include
        ${KERNELHEADERS_DIR}/arch/arm/include/generated
)

set(DRIVER_FILE cyclone_v_driver.ko)
set(KBUILD_CMD $(MAKE) -C ${KERNELHEADERS_DIR} modules M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR})

# Generate the Kbuild file through cmake.
FILE(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/Kbuild "obj-m := cyclone_v.o")

add_custom_command(OUTPUT ${DRIVER_FILE}
        COMMAND ${KBUILD_CMD}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS cyclone_v.c VERBATIM)

add_custom_target(driver ALL DEPENDS ${DRIVER_FILE})
# Parse c files
add_library(cyclone_v_driver_dummy cyclone_v.c)
