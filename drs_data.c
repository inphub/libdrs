/*
 * drs_proto_cmd.h
 *
 *  Created on: 1 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#include <unistd.h>
#include <assert.h>
#include <dap_common.h>

#include "commands.h"
#include "drs_data.h"
#include "drs_ops.h"

#define LOG_TAG "drs_data"

static unsigned int s_get_shift(unsigned int a_drs_num);

/**
 * @brief drs_data_get
 * @param a_drs                  ��������� �� ������ DRS
 * @param a_buffer               ����� ������, ������� DRS_PAGE_READ_SIZE �������
 * @param a_buffer_size          ������������ ������ ������ ��� ������ (��� ������)
 * @param a_flags                ����� ��������
 * @return 0 ���� �� ������ ����, -1 ���� ���
 */
int drs_data_get(drs_t * a_drs, unsigned short * a_buffer, int a_flags )
{
    assert(a_drs);
    assert(a_buffer);
    unsigned int l_ret=0,i=0;

    if(a_flags & DRS_OP_FLAG_EXT_START){
        log_it(L_INFO, "start ext DRS");
        drs_cmd(a_drs->id, ENABLE_EXT_PULSE);
    } else {
        drs_cmd(a_drs->id, START_DRS);
    }
    usleep(100);

    bool l_is_ready = false;
    bool l_loop = true;
    while( l_loop ) {
        l_is_ready = drs_get_flag_write_ready(a_drs->id);
        if( l_is_ready)
            break;

        i++;
        if( a_flags & DRS_OP_FLAG_EXT_START){
            if(i>100){
                l_loop = false;
                l_ret = -1;
            }
        }else{
            //if(ext_start==0){end=1;)
        }
        //readExternalStatus(0xc); //Peter fix
    }
    log_it(L_DEBUG, "drs_data_get achieved on step #%u, DRS is %s", i, l_is_ready ? "ready" : "not ready");
    if(l_is_ready ){
        drs_read_page(a_drs, a_drs->id, &a_buffer[a_drs->id * DRS_PAGE_SIZE]);
    }

    drs_set_flag_end_read(a_drs->id, true);
    return l_ret;
}

/**
 * @brief drs_read_page
 * @param a_drs           ������ DRS
 * @param a_page_num      ����� ��������
 * @param a_buffer        ��������� �� �����, �� ����� DRS_PAGE_READ_SIZE �������
 */
void drs_read_page(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer)
{
    assert(a_drs);
    assert(a_buffer);
    if ( a_drs->id ==0 )
        memcpy(a_buffer, &(((unsigned short *)data_map_drs1)[a_page_num*16384]), 0x8000);
    else
        memcpy(a_buffer, &(((unsigned short *)data_map_drs2)[a_page_num*16384]), 0x8000);
    a_drs->shift =s_get_shift( a_drs->id);
}


/**
 * unsigned int drsnum		����� drs ��� ����������� ������
 * return 					������ ������;
 */
static unsigned int s_get_shift(unsigned int a_drs_num)//npage
{
    unsigned short tmpshift;
    if (a_drs_num==0)
     tmpshift=((unsigned long *)data_map_shift_drs1)[0]&1023;
    else
     tmpshift=((unsigned long *)data_map_shift_drs2)[0]&1023;
    return tmpshift;
}