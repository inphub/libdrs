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

static bool s_debug_more=false;

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
        int l_ret = drs_data_get(&g_drs[d],0,(unsigned short *) (((byte_t *) a_buffer) + DRS_CELLS_COUNT *sizeof(unsigned short) ), DRS_CELLS_COUNT *sizeof(unsigned short)  );
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
    unsigned l_cmds = DRS_CMD_START_DRS;

    if (a_flags & DRS_OP_FLAG_EXT_START){
        log_it(L_INFO, "start ext DRS");
        l_cmds |= DRS_CMD_ENABLE_EXT_PULSE;
    }

    drs_cmd( -1, l_cmds);
    //usleep(100);

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
        debug_if(s_debug_more, L_DEBUG, "drs_data_get achieved on step #%u, DRS is %s", i, l_is_ready ? "ready" : "not ready");
    }else
        log_it(L_WARNING, "drs_data_get wasn't achieved after %u attempts, DRS is %s", i, l_is_ready ? "ready" : "not ready");

    if(a_flags & DRS_OP_FLAG_ROTATE)
        drs_read_page_rotated(a_drs, 0, a_buffer, a_buffer_size);
    else
        drs_read_page(a_drs, 0, a_buffer, a_buffer_size);

    drs_set_flag_end_read(a_drs->id, true);

#ifndef DRS_OPT_DATA_GET_NODELAYS
    usleep(DRS_PAGE_READ_DELAY);
#endif

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

    //log_it(L_DEBUG, "Global shift: %u , local shift: %u",drs_get_shift(a_drs->id), a_drs->shift);

}

/**
 * @brief drs_read_page_rotated
 * @param a_drs
 * @param a_page_num
 * @param a_buffer
 * @param a_buffer_size
 */
void drs_read_page_rotated(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size)
{
    assert(a_drs);
    assert(a_buffer);
    byte_t * a_drs_mem = a_drs->id == 0 ? (byte_t*)data_map_drs1 : (byte_t*)data_map_drs2;
    unsigned short * a_drs_page =(unsigned short *) (a_drs_mem + a_page_num*DRS_PAGE_READ_SIZE);
    a_drs->shift = s_get_shift( a_drs->id);
    unsigned int l_shift = DRS_CELLS_COUNT_BANK - a_drs->shift;

    unsigned int l_shift_global = drs_get_shift(a_drs->id);
    //log_it(L_DEBUG, "Global shift: %u , local shift: %u",l_shift_global, l_shift);


    size_t l_copy_size = (DRS_CELLS_COUNT_BANK * sizeof(unsigned short) * DRS_CHANNELS_COUNT);
    size_t l_total_size = 0;
    size_t l_in_offset, l_out_offset;

    byte_t * l_buf_in = (byte_t*) a_drs_page;
    byte_t * l_buf_out =(byte_t*) a_buffer;

    // Разворачиваем внутри банков
    for(unsigned b = 0; b < DRS_BANK_COUNT && l_total_size < a_buffer_size; b++){
        size_t l_bank_shift = b*DRS_CELLS_COUNT_BANK * DRS_CHANNELS_COUNT *sizeof (unsigned short); // смещение банка

        l_copy_size = (DRS_CELLS_COUNT_BANK - l_shift) * sizeof(unsigned short) * DRS_CHANNELS_COUNT;

        if (l_copy_size + l_total_size > a_buffer_size ) // Проверяем на предмет выхода за пределы буфера
            l_copy_size = a_buffer_size - l_total_size;
        // Копируем головной кусок
        l_out_offset = l_bank_shift;
        l_in_offset = l_bank_shift + l_shift*DRS_CHANNELS_COUNT*sizeof (unsigned short);
        memcpy( l_buf_out + l_out_offset,  l_buf_in + l_in_offset , l_copy_size ) ;
        l_total_size += l_copy_size;

        // проверяем, не всё ли это
        if(l_total_size >= a_buffer_size)
            break;


        // считаем размер хвостика
        unsigned l_copy_size_tail = ( (DRS_CELLS_COUNT_BANK * sizeof(unsigned short)* DRS_CHANNELS_COUNT) - l_copy_size);
        if ( (l_copy_size_tail + l_total_size) > a_buffer_size ) // Проверяем на предмет выхода за пределы буфера
            l_copy_size_tail = a_buffer_size - l_total_size;
        // копируем хвостик
        l_out_offset += l_copy_size ;
        l_in_offset = l_bank_shift;
        //memset( l_buf_out + l_out_offset, 0, l_copy_size_tail ) ;
        memcpy( l_buf_out + l_out_offset,
                l_buf_in + l_in_offset   , l_copy_size_tail ) ;
        l_total_size += l_copy_size_tail;

    }
}



/**
 * @brief drs_read_pages
 * @param a_drs
 * @param a_page_count
 * @param a_step
 * @param a_buffer
 * @param a_buffer_size
 */
void drs_read_pages(drs_t * a_drs, unsigned int a_page_count, unsigned int a_step,  unsigned short *a_buffer, size_t a_buffer_size)
{
    size_t l_offset = 0;
    bool l_loop = true;
    for (unsigned t=0; t< a_page_count && l_loop; t++){
        size_t l_read_size;
        if (l_offset + DRS_PAGE_READ_SIZE <= a_buffer_size)
            l_read_size = DRS_PAGE_READ_SIZE;
        else{
            l_read_size = a_buffer_size - l_offset;
            l_loop = false;
            log_it(L_ERROR, "Page read function goes out of input buffer, size %zd is not enought, requires %zd ( page read size %zd, num pages %u",
                    a_buffer_size, a_page_count * DRS_PAGE_READ_SIZE, DRS_PAGE_READ_SIZE, a_page_count );
        }
        drs_read_page(a_drs,t,&a_buffer[t*a_step], l_read_size);
    }
}



/**
 * unsigned int drsnum		номер drs для вычитывания сдвига
 * return 					индекс сдвига;
 */
static unsigned int s_get_shift(unsigned int a_drs_num)//npage
{
    return drs_get_shift(a_drs_num) &1023;
}

/**
 * @brief drs_get_shift
 * @param a_drs_num
 * @return
 */
unsigned int drs_get_shift(unsigned int a_drs_num)
{
    unsigned short tmpshift;
    if (a_drs_num==0)
     tmpshift=((unsigned long *)data_map_shift_drs1)[0];
    else
     tmpshift=((unsigned long *)data_map_shift_drs2)[0];

    return tmpshift;
}
