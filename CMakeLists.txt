cmake_minimum_required(VERSION 3.10)
project (drs)
set(CMAKE_C_STANDARD 11)

add_definitions("-D_GNU_SOURCE") 

option(DRS_OPT_DATA_GET_NODELAYS "Switch off all DRS data get delays " OFF)
option(DRS_BUILD_STATIC          "Static project build scheme" OFF)

find_package (Git)
if (GIT_FOUND)
      message("git found: ${GIT_EXECUTABLE} in version     ${GIT_VERSION_STRING}")
      execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} tag --list COMMAND tail -n 1 COMMAND tr -d '\n' OUTPUT_VARIABLE VERSION_MAJOR_MINOR )
      message("Last tag in list: ${VERSION_MAJOR_MINOR}")
      execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMAND ${GIT_EXECUTABLE} log --date-order --pretty=format:"%h" ${VERSION_MAJOR_MINOR}.. COMMAND wc -l COMMAND tr -d '\n' OUTPUT_VARIABLE VERSION_PATCH )

      execute_process(COMMAND echo ${VERSION_MAJOR_MINOR} COMMAND sed s/v// COMMAND sed s/\\..*//  COMMAND tr -d '\n' OUTPUT_VARIABLE VERSION_MAJOR )
      execute_process(COMMAND echo ${VERSION_MAJOR_MINOR} COMMAND sed s/v[0-9]*\\.// COMMAND tr -d '\n' OUTPUT_VARIABLE VERSION_MINOR )
else()
    SET(VERSION_MAJOR_MINOR "0.0")
    SET(VERSION_MAJOR "0")
    SET(VERSION_MINOR "0")
    SET(VERSION_PATCH "0z")
    message("Can't find GIT in your system for versioning")
endif (GIT_FOUND)

SET(VERSION_FULL "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}")

SET( CPACK_PACKAGE_NAME  "${PROJECT_NAME}")
SET( CPACK_PACKAGE_VERSION_MAJOR ${VERSION_MAJOR} )
SET( CPACK_PACKAGE_VERSION_MINOR ${VERSION_MINOR})
SET( CPACK_PACKAGE_VERSION_PATCH ${VERSION_PATCH})
SET( CPACK_GENERATOR "DEB")
SET( CPACK_SYSTEM_TYPE "${DEBIAN_OS_RELEASE_NAME}")
SET( CPACK_SYSTEM_VERSION "${DEBIAN_OS_VERSION}")
SET( CPACK_SYSTEM_CODENAME "${DEBIAN_OS_NAME}")

if(NOT CPACK_TARGET_ARCHITECTURE)
     set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE armel)
else()
     set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${CPACK_TARGET_ARCHITECTURE})
endif()

if(NOT CPACK_SYSTEM_NAME)
  set(CPACK_SYSTEM_NAME ${CPACK_SYSTEM_TYPE}-${CPACK_SYSTEM_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE})
endif()

 SET( CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}-${CPACK_PACKAGE_VERSION_PATCH}")

 if(CMAKE_BUILD_TYPE MATCHES Debug)
     SET( CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION}-dbg" )
 endif()

 SET( CPACK_DEBIAN_PACKAGE_MAINTAINER "Dmitriy Gerasimov <naeper@demlabs.net>" )
  SET( CPACK_DEBIAN_PACKAGE_DESCRIPTION_SUMMARY "libdrs" )
  SET( CPACK_DEBIAN_PACKAGE_DESCRIPTION   "libdrs" )
  SET( CPACK_DEBIAN_PACKAGE_SECTION "extra")
  SET( CPACK_DEBIAN_PACKAGE_DEPENDS "debconf (>=1.5), dpkg (>=1.17), bash (>=4), debconf-utils, dconf-cli")

  #have to figure out how to use it properly with custom RPATH for python plugins...
  if (NOT SUPPORT_PYTHON_PLUGINS)
      SET( CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
  endif()

  message("[*] Debian package setup pkgarch=${CPACK_DEBIAN_PACKAGE_ARCHITECTURE} version=${CPACK_PACKAGE_VERSION}")

#  SET( CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
#  "${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst;${CMAKE_CURRENT_SOURCE_DIR}/debian/prerm;${CMAKE_CURRENT_SOURCE_DIR}/debian/postrm;${CMAKE_CURRENT_SOURCE_DIR}/debian/templa
#tes;${CMAKE_CURRENT_SOURCE_DIR}/debian/config;")

message("Version ${VERSION_FULL}")

#possible need be setted during crosscompily
#by default it uses dpkg --print-architecture
#we ok with it on native builds.
#SET( CPACK_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
#SET( CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")


if ("${DAP_SDK_PATH}" STREQUAL "")
    message("DAP SDK is not specified, set default path")
    set(DAP_SDK_PATH "/opt/dap-sdk")
endif()

message("DAP SDK path: ${DAP_SDK_PATH}")

if ( NOT DAP_RELEASE AND NOT DAP_DEBUG)
    include(${DAP_SDK_PATH}/share/cmake/OS_Detection.cmake)
endif()

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

if(DRS_BUILD_STATIC OR BUILD_STATIC)
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
#INSTALL(FILES examples/* DESTINATION ${SHAREDIR}/examples )

