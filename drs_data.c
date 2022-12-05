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
#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"

#define LOG_TAG "drs_data"

static unsigned int s_get_shift(unsigned int a_drs_num);


/**
 * @brief drs_data_get_all
 * @param a_drs    Если NULL то он копирует для всех DRS
 * @param a_flags
 * @param a_buffer
 * @return
 */
int drs_data_get_all(drs_t * a_drs, int a_flags , unsigned short * a_buffer)
{
    if (a_drs){
        return drs_data_get(a_drs, a_flags, a_buffer, DRS_CELLS_COUNT *sizeof(unsigned short) );
    }else for (size_t d=0; d < DRS_COUNT; d++){
        int l_ret = drs_data_get(&g_drs[d],0,(unsigned short *) (((byte_t *) a_buffer) + DRS_PAGE_READ_SIZE), DRS_PAGE_READ_SIZE  );
        if (l_ret!= 0){
            log_it(L_ERROR,"data not read on DRS #%u", d);
            return l_ret;
        }
    }
    return 0;
};

/**
 * @brief drs_data_get
 * @param a_drs                  Указатель на объект DRS
 * @param a_flags                Флаги операции
 * @param a_buffer               буфер данных, минимум DRS_PAGE_READ_SIZE размера
 * @param a_buffer_size          максимальный размер данных для буфера (его размер)
 */
int drs_data_get(drs_t * a_drs, int a_flags, unsigned short * a_buffer, size_t a_buffer_size )
{
    assert(a_drs);
    assert(a_buffer);
    unsigned int l_ret=0,i=0;

    if(a_flags & DRS_OP_FLAG_EXT_START){
        log_it(L_INFO, "start ext DRS");
        drs_cmd(a_drs->id, ENABLE_EXT_PULSE);
    } else if (a_flags & DRS_OP_FLAG_CALIBRATE ){
        drs_cmd(a_drs->id, INIT_DRS | START_DRS );
        drs_cmd(a_drs->id, START_DRS);
    } else {
        drs_cmd(a_drs->id, START_DRS);
        //drs_cmd(a_drs->id, 1);
    }
    usleep(100);

    bool l_is_ready = false;
    bool l_loop = true;
    while( l_loop ) {
        l_is_ready = drs_get_flag_write_ready(a_drs->id);
        if( l_is_ready)
            break;

        i++;
//        if( a_flags & DRS_OP_FLAG_EXT_START){
            if(i>100){
                log_it(L_ERROR, "Was waiting for write_ready flag but without success");
                l_loop = false;
                l_ret = -1;
            }
//        }else{
            //if(ext_start==0){end=1;)
//        }
        //readExternalStatus(0xc); //Peter fix
    }
    if(l_is_ready ){
        log_it(L_DEBUG, "drs_data_get achieved on step #%u, DRS is %s", i, l_is_ready ? "ready" : "not ready");
    }else
        log_it(L_WARNING, "drs_data_get wasn't achieved after %u attempts, DRS is %s", i, l_is_ready ? "ready" : "not ready");

    drs_read_page(a_drs, 0, a_buffer, a_buffer_size);

    drs_set_flag_end_read(a_drs->id, true);
    return l_ret;
}


/**
 * @brief drs_read_page
 * @param a_drs
 * @param a_page_num
 * @param a_buffer
 * @param a_buffer_size
 */
void drs_read_page(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size)
{
    assert(a_drs);
    assert(a_buffer);
    if ( a_drs->id ==0 )
        memcpy(a_buffer, (unsigned short *) ( ((byte_t*)data_map_drs1 )+ a_page_num*DRS_PAGE_READ_SIZE), a_buffer_size ) ;
    else
       memcpy(a_buffer, (unsigned short *) ( ((byte_t*)data_map_drs2 )+ a_page_num*DRS_PAGE_READ_SIZE), a_buffer_size ) ;
    a_drs->shift =s_get_shift( a_drs->id);
}


/**
 * unsigned int drsnum		номер drs для вычитывания сдвига
 * return 					индекс сдвига;
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
