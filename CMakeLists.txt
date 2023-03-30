cmake_minimum_required(VERSION 3.10)
project (drs)
set(CMAKE_C_STANDARD 11)

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

if (NOT DRS_OWN_DAP_SDK)
        add_definitions("-DDAP_VERSION=\"\"")

        include(dap-sdk/cmake/OS_Detection.cmake)
        # Перечисление модулей DAP SDK которые нам нужны
        set(DAPSDK_MODULES "core crypto io app-cli")
        # Подключение DAP SDK
        add_subdirectory(dap-sdk)
endif()

#add_subdirectory(driver)

add_library(${PROJECT_NAME} SHARED ${DRS_SRCS} ${DRS_HEADERS})
add_library(${PROJECT_NAME}_static STATIC ${DRS_SRCS} ${DRS_HEADERS})
target_link_libraries(${PROJECT_NAME} dap_core dap_io dap_cli_server)
target_link_libraries(${PROJECT_NAME}_static dap_sdk_static)
target_include_directories(${PROJECT_NAME} INTERFACE .)
target_include_directories(${PROJECT_NAME}_static INTERFACE .)

if(BUILD_DAP_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

#--- Описываем действия при инсталляции
set(CMAKE_INSTALL_PREFIX /usr/local)
set(BINDIR ${CMAKE_INSTALL_PREFIX}/bin)
set(INCDIR ${CMAKE_INSTALL_PREFIX}/include)
set(CONFIGDIR ${CMAKE_INSTALL_PREFIX}/etc)
set(SBINDIR ${CMAKE_INSTALL_PREFIX}/sbin)
set(LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
set(RLIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
set(SHAREDIR ${CMAKE_INSTALL_PREFIX}/share)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${LIBDIR})
#I#NSTALL(PUBLIC_HEADER DESTINATION ${INCDIR})
#INSTALL(TARGETS ${PROJECT_NAME}_static drs_client DESTINATION ${LIBDIR})
