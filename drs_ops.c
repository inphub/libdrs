/*
 * drs_ops.c
 *
 *  Created on: 3 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#include <unistd.h>

#include <dap_common.h>

#include "drs.h"
#include "drs_ops.h"

#define LOG_TAG "drs_ops"

static s_debug_more = true;
/**
 * @brief drs_start
 */
void drs_start(int a_drs_num, int a_flags, unsigned a_pages_num)
{
    debug_if(s_debug_more, L_INFO,"drs_start() a_flags = 0x%08X", a_flags);
    drs_cmd( -1, a_flags);

    if(a_pages_num > 1){
        drs_reg_write(DRS_REG_NPAGES_MAX_DRS_A, a_pages_num);
        drs_reg_write(DRS_REG_NPAGES_MAX_DRS_B, a_pages_num);
        log_it(L_DEBUG, "Pages count %u", drs_reg_read(DRS_REG_NPAGES_MAX_DRS_A));
    }
}


/**
 * @brief drs_set_num_pages_all
 * @param a_num
 */
void drs_set_num_pages_all(unsigned int a_num)
{
    drs_reg_write(DRS_REG_NPAGES_MAX_DRS_A,a_num);
    drs_reg_write(DRS_REG_NPAGES_MAX_DRS_B,a_num);
    usleep(100);
}

/**
 * @brief drs_set_num_pages
 * @param a_drs
 * @param a_num
 */
void drs_set_num_pages(drs_t * a_drs, unsigned int a_num)
{
    drs_reg_write(DRS_BASE_NUM_PAGE + a_drs->id, a_num);
    usleep(100);
}


/**
 * @brief drs_set_flag_end_read
 * @param a_drs
 * @param a_enable
 */
void drs_set_flag_end_read(int a_drs_num, bool a_enable)
{
    switch (a_drs_num) {
        case -1:
            drs_reg_write(DRS_REG_READY_A, a_enable ? 1 : 0);
            drs_reg_write(DRS_REG_READY_B, a_enable? 1 : 0);
        break;
        case 0:
            drs_reg_write(DRS_REG_READY_A, a_enable ? 1 : 0);
        break;
        case 1:
            drs_reg_write(DRS_REG_READY_B, a_enable ? 1 : 0);
        break;
    }
   // usleep(100);
}

/**
 * @brief drs_get_flag_write_ready
 * @param l_drs_num
 * @return
 */
bool drs_get_flag_write_ready(int l_drs_num )
{
    usleep(100);
    //log_it(L_DEBUG, "get flag write ready for DRS %d", l_drs_num);
    switch(l_drs_num){
        case 0 : return drs_reg_read(DRS_REG_WAIT_DRS_A);
        case 1 : return drs_reg_read(DRS_REG_WAIT_DRS_B);
        default:
            log_it(L_ERROR, "Wrong DRS number, could be 0, 1 or -1 for both");
            return 0;
    }
}

/**
 * @brief drs_data_wait_for_ready
 * @param a_drs
 */
int drs_data_wait_for_ready(drs_t * a_drs)
{
    bool l_is_ready = false;
    bool l_loop = true;
    unsigned i = 0;
    while( l_loop ) {
        l_is_ready = drs_get_flag_write_ready(a_drs->id);
        if( l_is_ready)
            break;

        i++;
        if(i>100){
            log_it(L_ERROR, "Was waiting for write_ready flag but without success");
            return -1;
        }
    }
    return 0;
}


/**
 * @brief drs_cmd
 * @param a_drs_num
 * @param a_cmd
 */
void drs_cmd(int a_drs_num, unsigned int a_cmd)
{
    switch (a_drs_num){
        case -1:
            drs_reg_write(DRS_REG_CMD_DRS_1, a_cmd);
            drs_reg_write(DRS_REG_CMD_DRS_2, a_cmd);
        break;
        case 0:
            drs_reg_write(DRS_REG_CMD_DRS_1, a_cmd);
        break;
        case 1:
            drs_reg_write(DRS_REG_CMD_DRS_2, a_cmd);
        break;
        default:
            log_it(L_ERROR, "Wrong DRS number, could be 0, 1 or -1 for both");
    }
    usleep(100);
}

/**
 * @brief drs_set_sinus_signal
 * @param a_sinus_signal
 */
void drs_set_sinus_signal(bool a_sinus_signal)
{
    drs_reg_write( DRS_REG_CALIB_SIN_ON_CH9, a_sinus_signal ? 0 : 1 );
    usleep(100);
}
