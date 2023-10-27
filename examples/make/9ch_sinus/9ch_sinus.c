#include <unistd.h>

#include <dap_common.h>
#include <dap_config.h>
#include <dap_events.h>
#include <dap_cli_server.h>

#include <dap_sdk.h>

#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"

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
    dap_set_appname("9ch_sinus");
    dap_sdk_init(NULL);
    //dap_sdk_parse_args(argc, argv);

    log_it(L_DEBUG, "DRS init...");
    if(drs_init(0) != 0){
        log_it(L_CRITICAL, "Can't init drs");
        return -12;
    }

    drs_set_mode(-1,DRS_MODE_CAL_TIME);
    drs_set_sinus_signal(true);

    return 0;
}
