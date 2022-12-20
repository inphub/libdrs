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

/**
 * @brief drs_start
 */
void drs_start(int a_drs_num)
{
    if(a_drs_num == -1){
        write_reg(DRS_REG_CMD_DRS_1, 2);
        write_reg(DRS_REG_CMD_DRS_2, 2);
    }else if (a_drs_num >=0)
        write_reg(DRS_REG_CMD_DRS_1 + a_drs_num, 2);
    else
        log_it(L_ERROR, "Wrong DRS num %d", a_drs_num);
    usleep(20);
}


/**
 * @brief drs_set_num_pages_all
 * @param a_num
 */
void drs_set_num_pages_all(unsigned int a_num)
{
    write_reg(DRS1_NUM_PAGE,a_num);
    write_reg(DRS2_NUM_PAGE,a_num);
    usleep(100);
}

/**
 * @brief drs_set_num_pages
 * @param a_drs
 * @param a_num
 */
void drs_set_num_pages(drs_t * a_drs, unsigned int a_num)
{
    write_reg(DRS_BASE_NUM_PAGE + a_drs->id, a_num);
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
            write_reg(DRS_REG_READY_A, a_enable ? 1 : 0);
            write_reg(DRS_REG_READY_B,a_enable? 1 : 0);
        break;
        case 0:
            write_reg(DRS_REG_READY_A, a_enable ? 1 : 0);
        break;
        case 1:
            write_reg(DRS_REG_READY_B, a_enable ? 1 : 0);
        break;
    }
    usleep(100);
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
        case 0 : return read_reg(DRS_REG_WAIT_DRS_A);
        case 1 : return read_reg(DRS_REG_WAIT_DRS_B);
        default:
            log_it(L_ERROR, "Wrong DRS number, could be 0, 1 or -1 for both");
            return 0;
    }
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
            write_reg(DRS_REG_CMD_DRS_1, a_cmd);
            write_reg(DRS_REG_CMD_DRS_2, a_cmd);
        break;
        case 0:
            write_reg(DRS_REG_CMD_DRS_1, a_cmd);
        break;
        case 1:
            write_reg(DRS_REG_CMD_DRS_2, a_cmd);
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
    write_reg( DRS_REG_CALIB_SIN_ON, a_sinus_signal ? 1 : 0 );
    usleep(100);
}
