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
        write_reg(DRS1_CONTROL_REG, 2);
        write_reg(DRS2_CONTROL_REG, 2);
    }else if (a_drs_num >=0)
        write_reg(DRS_BASE_CONTROL_REG + a_drs_num, 2);
    else
        log_it(L_ERROR, "Wrong DRS num %d", a_drs_num);
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
            write_reg(21, a_enable ? 1 : 0);
            write_reg(22,a_enable? 1 : 0);
        break;
        case 0:
            write_reg(21, a_enable ? 1 : 0);
        break;
        case 1:
            write_reg(22, a_enable ? 1 : 0);
        break;
    }
    usleep(100);
}

/**
 * @brief drs_get_flag_write_ready
 * @param l_drs_num
 * @return
 */
unsigned int drs_get_flag_write_ready(int l_drs_num )
{
    usleep(100);
//    printf("read: adr=0x%08x, val=0x%08x\n", DRS_MODE_REG, mode), fflush(stdout);
    switch(l_drs_num){
        case -1: return read_reg(49) & read_reg(50) ;
        case 0 : return read_reg(49);
        case 1 : return read_reg(50);
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
void drs_cmd(unsigned int a_drs_num, unsigned int a_cmd)
{
    switch (a_drs_num){
        case -1:
            write_reg(DRS1_CONTROL_REG, a_cmd);
            write_reg(DRS2_CONTROL_REG, a_cmd);
        break;
        case 0:
            write_reg(DRS1_CONTROL_REG, a_cmd);
        break;
        case 1:
            write_reg(DRS2_CONTROL_REG, a_cmd);
        break;
        default:
            log_it(L_ERROR, "Wrong DRS number, could be 0, 1 or -1 for both");
    }
    usleep(100);
}

