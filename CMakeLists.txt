cmake_minimum_required(VERSION 3.10)
project (drs)
set(CMAKE_C_STANDARD 11)

add_definitions("-D_GNU_SOURCE") 

option(ADC_OPT_DATA_GET_NODELAYS "Switch off all DRS data get delays " OFF)
option(ADC_BUILD_STATIC          "Static project build scheme" OFF)

if ("${DAP_SDK_PATH}" STREQUAL "")
        message("DAP SDK is not specified, set default path")
        set(DAP_SDK_PATH "/opt/dap-sdk")
endif()

message("DAP SDK path: ${DAP_SDK_PATH}")

include(${DAP_SDK_PATH}/share/cmake/OS_Detection.cmake)


if (DRS_OPT_DATA_GET_NODELAYS)
    add_definitions("-DDRS_OPT_DATA_GET_NODELAYS")
    message("[!] DRS data get no delay mode")
endif()

file(GLOB DRS_SRCS 
      *.c 
)
file(GLOB DRS_HEADERS 
      *.h 
)

set(BINDIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INCDIR ${CMAKE_INSTALL_PREFIX}/include)
set(CONFIGDIR ${CMAKE_INSTALL_PREFIX}/etc)
set(SBINDIR ${CMAKE_INSTALL_PREFIX}/sbin)
set(LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
set(RLIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
set(SHAREDIR ${CMAKE_INSTALL_PREFIX}/share)
include_directories(${DAP_SDK_PATH}/include)

#add_subdirectory(driver)

if(ADC_BUILD_STATIC)
        message("Build static library ${PROJECT_NAME}")
        add_library(${PROJECT_NAME} STATIC ${DRS_SRCS} ${DRS_HEADERS})
else()
        message("Build shared library ${PROJECT_NAME}")
        add_library(${PROJECT_NAME} SHARED ${DRS_SRCS} ${DRS_HEADERS})
endif()


target_link_directories(${PROJECT_NAME} PUBLIC ${DAP_SDK_PATH}/lib )
target_link_libraries(${PROJECT_NAME} dap_sdk)

target_include_directories(${PROJECT_NAME} INTERFACE . )
if(BUILD_DAP_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${LIBDIR})
INSTALL(FILES ${DRS_HEADERS} DESTINATION ${INCDIR})

