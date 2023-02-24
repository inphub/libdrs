cmake_minimum_required(VERSION 3.10)
project (drs)

add_definitions("-D_GNU_SOURCE") 

option(DRS_OPT_DATA_GET_NODELAYS "Switch off all DRS data get delays " OFF)

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

add_library(${PROJECT_NAME} STATIC ${DRS_SRCS} ${DRS_HEADERS})
#add_library(${PROJECT_NAME} SHARED ${DRS_SRCS} ${DRS_HEADERS})
target_link_libraries(${PROJECT_NAME} dap_core)
target_include_directories(${PROJECT_NAME} INTERFACE .)

if(BUILD_DAP_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
