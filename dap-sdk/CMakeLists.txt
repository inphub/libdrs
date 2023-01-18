set(DAP_SDK_NATIVE_VERSION "2.2-2")

# We switch off SSL until its not switched on back
if(DAPSDK_MODULES MATCHES "ssl-support")
    add_definitions("-DDAP_NET_CLIENT_SSL")
else()
    add_definitions("-DDAP_NET_CLIENT_NO_SSL")
endif()

# Core, add it always
add_subdirectory(modules/core)
if (DAPSDK_MODULES MATCHES "crypto")
    # Cryptography
    add_subdirectory(modules/crypto)
endif()

if (DAPSDK_MODULES MATCHES "app-cli")
    add_subdirectory(modules/net/app-cli)
    add_subdirectory(modules/net/server/cli_server)
endif()

if (DAPSDK_MODULES MATCHES "plugin")
    add_subdirectory(plugin)
endif()

# I/O subsystem
if (DAPSDK_MODULES MATCHES "io")
    add_subdirectory(modules/io)
endif()

# Networking core
if (DAPSDK_MODULES MATCHES "network-core")
    add_subdirectory(modules/net/stream)
endif()

if (DAPSDK_MODULES MATCHES "network-pure")
    add_subdirectory(modules/io)
endif()

# Networking client
if (DAPSDK_MODULES MATCHES "network-client")
    add_subdirectory(modules/net/client)
endif()

# Networking server
if (DAPSDK_MODULES MATCHES "network-server")
    add_subdirectory(modules/net/server)
#    add_subdirectory(net/server-udp)
endif()
