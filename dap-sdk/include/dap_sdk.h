/*
 * dap_sdk.h
 *
 *  Created on: 21 March 2023
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once
#include <dap_common.h>
typedef struct dap_config dap_config_t;
typedef struct dap_server dap_server_t;
typedef struct dap_vars{
    bool debug_mode;
    struct {
        char * sys_dir;
        dap_config_t *config;
    } core;
    struct{
      size_t threads_count;
      const char * pid_file_path;
    } sys;
    struct {
        struct{
            bool enabled;
            dap_server_t * tcp;
        } server;
        struct {
            bool debug_more;
        } cli;
    } io;
    struct {

    } crypto;
} dap_vars_t;
extern dap_vars_t g_dap_vars;

int dap_sdk_init(const char *a_json_args,... );
int dap_sdk_parse_args( int argc, const char **argv );
void dap_sdk_deinit();
