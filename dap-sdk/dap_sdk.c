/*
 * dap_sdk.h
 *
 *  Created on: 21 March 2023
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <dap_common.h>
#include <dap_config.h>
#include <dap_strfuncs.h>
#include <dap_file_utils.h>
#include <dap_process_manager.h>
#include <dap_server.h>
#include <dap_events.h>
#include <dap_proc_thread.h>
#include <dap_strfuncs.h>
#include <dap_file_utils.h>
#include <dap_cli_server.h>
#include "sig_unix_handler.h"

#include "dap_sdk.h"

#define LOG_TAG "dap-sdk"

dap_vars_t g_dap_vars = {0};

// Path to the PID file
static void s_exit_if_server_already_running( void );

/**
 * @brief dap_sdk_init
 * @param a_json_args
 * @return
 */
int dap_sdk_init(const char * a_json_args,... )
{
    UNUSED(a_json_args); // TODO сделать парсинг
    // Базовый путь к папке приложения
    g_dap_vars.core.sys_dir = g_sys_dir_path = dap_strdup_printf("/opt/%s", dap_get_appname());

    // Путь к логам, если его нет, то создаёт его
    // char *l_log_dir = dap_strdup_printf("%s/var/log", g_sys_dir_path);
    //dap_mkdir_with_parents(l_log_dir);
    //char * l_log_file = dap_strdup_printf( "%s/%s.log", l_log_dir, dap_get_appname());

    // Инициализирует core модуль DAP SDK
    if (dap_common_init(dap_get_appname(), NULL, NULL) != 0) {
        printf("Fatal Error: Can't init common functions module");
        return -2;
    }

    // удаляем и обнуляем неиспользуемые переменные
    //DAP_DEL_Z(l_log_dir);
    //DAP_DEL_Z(l_log_file);

    // Инициализируем работу с конфигами
    char l_config_dir[MAX_PATH];
    l_config_dir[0] = '\0';
    snprintf(l_config_dir,MAX_PATH, "%s/etc", g_sys_dir_path);
    dap_config_init(l_config_dir);
    g_dap_vars.core.config = g_config = dap_config_open(dap_get_appname());
    if (!g_config) {
        return -1;
    }

    log_it( L_DAP, "*** DRS Server version: %s ***", DAP_VERSION );

    // change to dap_config_get_item_int_default when it's will be possible
    size_t l_thread_cnt = 0;

    const char *s_thrd_cnt = dap_config_get_item_str( g_config, "general", "threads_limit" );
    if ( s_thrd_cnt != NULL )
        g_dap_vars.sys.threads_count = l_thread_cnt = (size_t)atoi( s_thrd_cnt );

    if ( !l_thread_cnt ) {
        #ifndef _WIN32
            l_thread_cnt = (size_t)sysconf(_SC_NPROCESSORS_ONLN);
        #else
            SYSTEM_INFO si;
            GetSystemInfo( &si );
            l_thread_cnt = si.dwNumberOfProcessors;
        #endif
    }


    g_dap_vars.sys.pid_file_path = dap_config_get_item_str_default( g_config,  "general", "pid_path","/tmp") ;

    g_dap_vars.debug_mode = dap_config_get_item_bool_default( g_config,"general","debug_mode", false );
    //  bDebugMode = true;//dap_config_get_item_bool_default( g_config,"general","debug_mode", false );

    if ( g_dap_vars.debug_mode )
        log_it( L_ATT, "*** DEBUG MODE ***" );
    else
       log_it( L_ATT, "*** NORMAL MODE ***" );

    dap_log_level_set( g_dap_vars.debug_mode ? L_DEBUG : L_NOTICE );

    // New event loop init
    dap_events_init(l_thread_cnt, 3600*24*365);
    dap_events_start();
    dap_proc_thread_init(l_thread_cnt);
    usleep(2000);

    g_dap_vars.io.server.enabled = dap_config_get_item_bool_default( g_config, "server", "enabled", false );

    if ( g_dap_vars.io.server.enabled && dap_server_init() != 0 ) {
        log_it( L_CRITICAL, "Can't init socket server module" );
        return -4;
    }

    // Инициализируем CLI сервер

    bool l_cli_enabled = dap_config_get_item_bool_default( g_config, "cli", "enabled", true );
    if (l_cli_enabled) {
        g_dap_vars.io.cli.debug_more = dap_config_get_item_bool_default(g_config,"cli","debug_more",false);


        uint16_t l_listen_port = dap_config_get_item_uint16_default( g_config, "cli", "listen_port_tcp",0); // For backward compatibility
        if(l_listen_port == 0)
            l_listen_port = dap_config_get_item_uint16_default( g_config, "cli", "listen_port",0);

        if ( dap_cli_server_init( g_dap_vars.io.cli.debug_more,
                             l_listen_port ? dap_config_get_item_str(g_config, "cli", "listen_address")
                                           : dap_config_get_item_str( g_config, "cli", "listen_unix_socket_path"),
                             l_listen_port, dap_config_get_item_str( g_config, "cli", "listen_unix_socket_permissions")
                            ) ) {
            log_it( L_CRITICAL, "Can't init CLI server" );
            return -11;
        }
    }


    // Инициируем обработчики юникс сигналов
    if (sig_unix_handler_init(dap_config_get_item_str_default(g_config,
                                                              "general",
                                                              "pid_path",
                                                              "/tmp")) != 0) {
        log_it(L_CRITICAL,"Can't init sig unix handler module");
        return -12;
    }

    save_process_pid_in_file( g_dap_vars.sys.pid_file_path);

    dap_server_t *l_server = NULL;
    if ( g_dap_vars.io.server.enabled ) {

        int32_t l_port = dap_config_get_item_int32(g_config, "server", "listen_port_tcp");

        if( l_port > 0 ) {
            l_server = dap_server_new( (dap_config_get_item_str(g_config, "server", "listen_address")),
                                      (uint16_t) l_port, SERVER_TCP, NULL );
        } else
            log_it( L_WARNING, "Server is enabled but no port is defined" );

    }
    return 0;
}

void dap_sdk_deinit()
{
    // Deinit modules
    dap_events_deinit();
    dap_config_close( g_config );
    dap_interval_timer_deinit();
    dap_common_deinit();

}

/**
 * @brief Разбирает параметры командной строкиы
 * @param argc
 * @param argv
 */
int dap_sdk_parse_args( int argc, const char **argv )
{
    bool l_is_daemon = false;
    if( argc>=2){
        if (strcmp(argv[1], "--stop") == 0) {
            pid_t pid = get_pid_from_file(g_dap_vars.sys.pid_file_path);

            if ( pid == 0 ) {
                log_it( L_ERROR, "Can't read pid from file" );
                exit( -20 );
            }

            if ( kill_process(pid) ) {
                log_it( L_INFO, "Server successfully stopped" );
                exit( 0 );
            }

            log_it( L_WARNING, "Server not stopped. Maybe he is not running now?" );
            exit( -21 );
        } else if (strcmp(argv[1], "-D") == 0) {
            log_it( L_INFO, "Daemonize server starting..." );
            s_exit_if_server_already_running( );
            l_is_daemon = true;
            daemonize_process( );
        } else {
            log_it( L_WARNING, "Unknown option from command line" );
        }
    }

    if( !l_is_daemon )
        s_exit_if_server_already_running( );

    return 0;
}

/**
 * @brief exit_if_server_already_running
 */
static void s_exit_if_server_already_running( void )
{
    pid_t pid = get_pid_from_file(g_dap_vars.sys.pid_file_path);
    bool  mf = false;

    if ( (pid != 0 && is_process_running(pid)) || mf ) {
        log_it( L_WARNING, "Proccess %"DAP_UINT64_FORMAT_U" is running, don't allow "
                            "to run more than one copy of DapServer, exiting...", (uint64_t)pid );

        sleep(1);
        exit( -2 );
    }
}

