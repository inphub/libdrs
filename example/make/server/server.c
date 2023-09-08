#include <unistd.h>

#include <dap_common.h>
#include <dap_config.h>
#include <dap_events.h>
#include <dap_cli_server.h>

#include <dap_sdk.h>

#include "drs.h"
#include "drs_proto.h"

// Tag for log_it and debug_if macros
#define LOG_TAG "drs_server"

static int s_callback_drs_server(int a_argc, char ** a_argv, char **a_str_reply);

/**
 * @brief Точка входа в сервер
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, const char **argv)
{
    // Имя приложения
    dap_set_appname("drs_server");
    dap_sdk_init(NULL);
    //dap_sdk_parse_args(argc, argv);

    log_it(L_DEBUG, "DRS init...");
    if(drs_init(0) != 0){
        log_it(L_CRITICAL, "Can't init drs protocol");
        return -12;
    }

    // Инициализация протокола
    if(drs_proto_init( g_dap_vars.io.server.tcp ) != 0){
        log_it(L_CRITICAL, "Can't init drs protocol");
        return -13;
    }

    // Добавление кастомной команды
    dap_cli_server_cmd_add ("drs_server", s_callback_drs_server, "DRS server brief",
                "DRS Server brief output command\n"
             );


    // Вечный цикл, пока приложение работает
    int l_rc = 0;
    l_rc = dap_events_wait();
    log_it( l_rc ? L_CRITICAL : L_NOTICE, "Server loop stopped with return code %d", l_rc );

    // Деинициализация
    dap_sdk_deinit();
    drs_proto_deinit();
    drs_deinit();

    return l_rc * 10;
}

/**
 * @brief s_callback_init
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return
 */
static int s_callback_drs_server(int a_argc, char ** a_argv, char **a_str_reply)
{
    UNUSED(a_argc);
    UNUSED(a_argv);
    dap_cli_server_cmd_set_reply_text(a_str_reply, "DRS Server");
    return 0;
}
