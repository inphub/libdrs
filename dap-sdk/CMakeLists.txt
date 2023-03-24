cmake_minimum_required(VERSION 3.10)
set(DAP_SDK_NATIVE_VERSION "3.0-0")
project (dap_sdk)

file(GLOB DAP_SDK_SRCS
      *.c
)
file(GLOB DAP_SDK_HEADERS
      include/*.h
)

set(DAP_SDK_SRC_EXT "")
set(DAP_SDK_HEADERS_EXT "")
include_directories(include)
if(UNIX)
        set(DAP_SDK_SRC_EXT "${DAP_SDK_SRC_EXT};${PROJECT_SOURCE_DIR}/os/unix/sig_unix_handler.c")
        set(DAP_SDK_HEADERS_EXT "${DAP_SDK_HEADERS_EXT};${PROJECT_SOURCE_DIR}/os/unix/sig_unix_handler.h")
        include_directories(os/unix)
endif()
message("Modules: \"${DAP_SDK_MODULES_LIBS}\"")

# We switch off SSL until its not switched on back
if(DAPSDK_MODULES MATCHES "ssl-support")
    add_definitions("-DDAP_NET_CLIENT_SSL")
else()
    add_definitions("-DDAP_NET_CLIENT_NO_SSL")
endif()

# Core, add it always
set(DAP_SDK_MODULES_LIBS "dap_core")
add_subdirectory(modules/core)

add_library(${PROJECT_NAME} SHARED ${DAP_SDK_SRCS} ${DAP_SDK_SRC_EXT} ${DAP_SDK_HEADERS} ${DAP_SDK_HEADERS_EXT})
add_library(${PROJECT_NAME}_static STATIC ${DAP_SDK_SRCS} ${DAP_SDK_SRC_EXT} ${DAP_SDK_HEADERS} ${DAP_SDK_HEADERS_EXT})
target_link_libraries(${PROJECT_NAME} dap_core)
target_include_directories(${PROJECT_NAME} INTERFACE . include/)
target_link_libraries(${PROJECT_NAME}_static dap_core)
target_include_directories(${PROJECT_NAME}_static INTERFACE . include/)

if (DAPSDK_MODULES MATCHES "crypto")
    # Cryptography
    add_subdirectory(modules/crypto)
    target_link_libraries(${PROJECT_NAME} dap_crypto)
    target_link_libraries(${PROJECT_NAME}_static dap_crypto)
endif()

if (DAPSDK_MODULES MATCHES "app-cli")
    add_subdirectory(modules/net/app-cli)
    add_subdirectory(modules/net/server/cli_server)
    target_link_libraries(${PROJECT_NAME} dap_cli_server)
    target_link_libraries(${PROJECT_NAME}_static dap_cli_server)
endif()

if (DAPSDK_MODULES MATCHES "plugins")
    add_subdirectory(plugins)
    set(DAP_SDK_MODULES_LIBS "${DAP_SDK_MODULES_LIBS} dap_plugins")
endif()

# I/O subsystem
if (DAPSDK_MODULES MATCHES "io")
    add_subdirectory(modules/io)
    target_link_libraries(${PROJECT_NAME} dap_io)
    target_link_libraries(${PROJECT_NAME}_static dap_io)
endif()

# Networking core
if (DAPSDK_MODULES MATCHES "network-core")
    add_subdirectory(modules/net/stream)
    set(DAP_SDK_MODULES_LIBS "${DAP_SDK_MODULES_LIBS} libdap-stream")
endif()

# Networking client
if (DAPSDK_MODULES MATCHES "network-client")
    add_subdirectory(modules/net/client)
    set(DAP_SDK_MODULES_LIBS "${DAP_SDK_MODULES_LIBS} dap_client")
endif()

# Networking server
if (DAPSDK_MODULES MATCHES "network-server")
    add_subdirectory(modules/net/server)
    set(DAP_SDK_MODULES_LIBS "${DAP_SDK_MODULES_LIBS} libdap-server")
#    add_subdirectory(net/server-udp)
endif()

