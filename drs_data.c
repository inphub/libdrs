/*
 * drs_proto_cmd.h
 *
 *  Created on: 1 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#include <sys/stat.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

#include <dap_sdk.h>
#include <dap_common.h>
#include <dap_file_utils.h>
#include <dap_strfuncs.h>
#include <dap_string.h>
#include <dap_config.h>

#include "commands.h"
#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"

#define LOG_TAG "drs_data"

static bool s_debug_more=false;

/**
 * @brief drs_data_init
 * @return
 */
int drs_data_init()
{
    s_debug_more = dap_config_get_item_bool_default(g_config,"drs_data","debug_more", false);
}

/**
 * @brief drs_data_deinit
 */
void drs_data_deinit()
{

}


/**
 * @brief drs_data_get_all
 * @param a_drs    Если NULL то он копирует для всех DRS
 * @param a_flags
 * @param a_buffer
 * @return
 */
int drs_data_get_page_first(drs_t * a_drs, int a_flags , unsigned short * a_buffer)
{
    if (a_drs){
        return drs_data_get_page_from_first(a_drs, a_flags, a_buffer, DRS_CELLS_COUNT *sizeof(unsigned short) );
    }else for (size_t d=0; d < DRS_COUNT; d++){
        int l_ret = drs_data_get_page_from_first(&g_drs[d],0,(unsigned short *) (((byte_t *) a_buffer) + DRS_CELLS_COUNT *sizeof(unsigned short) ), DRS_CELLS_COUNT *sizeof(unsigned short)  );
        if (l_ret!= 0){
            log_it(L_ERROR,"data not read on DRS #%u", d);
            return l_ret;
        }
    }
    return 0;
};


/**
 * @brief drs_data_get_page
 * @param a_drs
 * @param a_flags
 * @param a_page
 * @param a_buffer
 * @param a_buffer_size
 * @return
 */
int drs_data_get_page(drs_t * a_drs, int a_flags ,unsigned a_page, unsigned short * a_buffer, size_t a_buffer_size)
{
    assert(a_drs);
    assert(a_buffer);
    unsigned int l_ret=0,i=0;
    bool l_do_commands = a_flags & DRS_OP_FLAG_SOFT_START || a_flags & DRS_OP_FLAG_EXT_START;

    if (l_do_commands ){
        unsigned l_cmds = 0;

        if (a_flags & DRS_OP_FLAG_EXT_START){
            log_it(L_INFO, "start ext DRS");
            l_cmds |= DRS_CMD_EXT_START;
        } else{
            l_cmds |= DRS_CMD_SOFT_START;
        }

        drs_cmd( -1, l_cmds);
        //drs_cmd( a_drs->id, l_cmds);

        bool l_is_ready = drs_data_wait_for_ready(a_drs) == 0;
        if(l_is_ready ){
            debug_if(s_debug_more, L_DEBUG, "drs_data_get achieved on step #%u, DRS is %s", i, l_is_ready ? "ready" : "not ready");
        }else{
            log_it(L_WARNING, "drs_data_get wasn't achieved after %u attempts, DRS is %s", i, l_is_ready ? "ready" : "not ready");
            //return -1;
        }
    }

    if(a_flags & DRS_OP_FLAG_ROTATE)
        drs_read_page_rotated(a_drs, a_page, a_buffer, a_buffer_size);
    else
        drs_read_page(a_drs, a_page, a_buffer, a_buffer_size);

//#ifndef DRS_OPT_DATA_GET_NODELAYS
    //usleep(DRS_PAGE_READ_DELAY);
//#endif
    if( l_ret == 0 && l_do_commands ){
        drs_set_flag_end_read(a_drs->id, true);
    }


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
        memcpy(a_buffer,  ((byte_t*)data_map_drs1 )+ a_page_num*DRS_PAGE_READ_SIZE, a_buffer_size ) ;
    else
       memcpy(a_buffer, ((byte_t*)data_map_drs2 )+ a_page_num*DRS_PAGE_READ_SIZE, a_buffer_size ) ;
    a_drs->shift =drs_get_shift( a_drs->id, a_page_num);
    a_drs->shift_bank =a_drs->shift & 1023;

    //log_it(L_DEBUG, "Global shift: %u , local shift: %u",drs_get_shift(a_drs->id), a_drs->shift);

}


void drs_data_rotate_bank(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size)
{
    assert(a_drs);
    assert(a_mem_in);
    assert(a_mem_out);
    assert(a_cell_size);
    unsigned int l_shift_global = a_drs->shift;
    unsigned int l_shift = DRS_CELLS_COUNT_BANK - a_drs->shift_bank;

    //log_it(L_DEBUG, "Global shift: %u , local shift: %u",l_shift_global, l_shift);

    size_t l_total_size = 0;
    size_t l_in_offset, l_out_offset;

    const byte_t * l_buf_in = (byte_t*) a_mem_in;
    byte_t * l_buf_out =  DAP_NEW_STACK_SIZE(byte_t, a_mem_size);
    memset(l_buf_out,0, a_mem_size);

    // Разворачиваем внутри банков
    for(unsigned b = 0; b < DRS_CHANNEL_BANK_COUNT && l_total_size < a_mem_size; b++){
        size_t l_bank_shift = b*DRS_CELLS_COUNT_BANK * DRS_CHANNELS_COUNT *a_cell_size; // смещение банка
        size_t l_copy_size = (DRS_CELLS_COUNT_BANK - l_shift) * a_cell_size * DRS_CHANNELS_COUNT;

        if (l_copy_size + l_total_size > a_mem_size ) // Проверяем на предмет выхода за пределы буфера
            l_copy_size = a_mem_size - l_total_size;
        // Копируем головной кусок
        l_out_offset = l_bank_shift;
        l_in_offset = l_bank_shift + l_shift*DRS_CHANNELS_COUNT*a_cell_size;
        if(l_copy_size){
            memcpy( l_buf_out + l_out_offset,  l_buf_in + l_in_offset , l_copy_size ) ;
            l_total_size += l_copy_size;
        }

        // проверяем, не всё ли это
        if(l_total_size >= a_mem_size)
            break;


        // считаем размер хвостика
        unsigned l_copy_size_tail = ( (DRS_CELLS_COUNT_BANK * a_cell_size* DRS_CHANNELS_COUNT) - l_copy_size);
        if ( (l_copy_size_tail + l_total_size) > a_mem_size ) // Проверяем на предмет выхода за пределы буфера
            l_copy_size_tail = a_mem_size - l_total_size;
        // копируем хвостик
        l_out_offset += l_copy_size ;
        l_in_offset = l_bank_shift;
        //memset( l_buf_out + l_out_offset, 0, l_copy_size_tail ) ;
        memcpy( l_buf_out + l_out_offset,
                l_buf_in + l_in_offset   , l_copy_size_tail ) ;
        l_total_size += l_copy_size_tail;

    }

    memcpy(a_mem_out, l_buf_out,a_mem_size );

}

/**
 * @brief drs_data_rotate_global
 * @param a_drs
 * @param a_mem_in
 * @param a_mem_out
 * @param a_mem_size
 * @param a_cell_size
 */
void drs_data_rotate_global(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size)
{
  assert(a_drs);
  assert(a_mem_in);
  assert(a_mem_out);
  assert(a_cell_size);
  unsigned int l_shift_global = a_drs->shift;
  byte_t * l_buf_out =  DAP_NEW_STACK_SIZE(byte_t, a_mem_size);
  memset(l_buf_out,0, a_mem_size);

  //memcpy(a_mem_out, l_buf_out, a_mem_size);
  // Разворачиваем глобально всё
  unsigned l_head_size = (DRS_CELLS_COUNT_CHANNEL - l_shift_global) * a_cell_size * DRS_CHANNELS_COUNT;
  memcpy(l_buf_out, a_mem_in + l_shift_global* a_cell_size * DRS_CHANNELS_COUNT , l_head_size  );
  memcpy(l_buf_out + l_head_size, a_mem_in, l_shift_global * a_cell_size * DRS_CHANNELS_COUNT );

  memcpy(a_mem_out, l_buf_out,a_mem_size );
}


void drs_data_rotate_bank9(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size)
{
  assert(a_drs);
  assert(a_mem_in);
  assert(a_mem_out);
  assert(a_cell_size);
  unsigned int l_shift = a_drs->shift_bank+1;
  byte_t * l_buf_out =  DAP_NEW_STACK_SIZE(byte_t, a_mem_size );
  memset(l_buf_out,0, a_mem_size);

  //memcpy(a_mem_out, l_buf_out, a_mem_size);
  // Разворачиваем 9ый канал
  unsigned l_head_size = (DRS_CELLS_COUNT_BANK - l_shift) * a_cell_size * DRS_CHANNELS_COUNT;
  memcpy(l_buf_out, ((byte_t*) a_mem_in)  + l_shift* a_cell_size * DRS_CHANNELS_COUNT , l_head_size  );
  memcpy(l_buf_out + l_head_size, a_mem_in ,
         l_shift * a_cell_size * DRS_CHANNELS_COUNT );

  memcpy(a_mem_out, l_buf_out,a_mem_size );
}




/**
 * @brief drs_read_page_rotated
 * @param a_drs
 * @param a_page_num
 * @param a_buffer
 * @param a_buffer_size
 */
void drs_read_page_rotated(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer,const size_t a_buffer_size)
{
    assert(a_drs);
    assert(a_buffer);
    byte_t * a_drs_mem = a_drs->id == 0 ? (byte_t*)data_map_drs1 : (byte_t*)data_map_drs2;
    unsigned short * a_drs_page =(unsigned short *) (a_drs_mem + a_page_num*DRS_PAGE_READ_SIZE);

    drs_data_rotate_bank(a_drs, a_drs_page, a_buffer, a_buffer_size, sizeof(unsigned short));
}



/**
 * @brief drs_read_pages
 * @param a_drs
 * @param a_page_count
 * @param a_offset
 * @param a_buffer
 * @param a_buffer_size
 */
int drs_read_pages(drs_t * a_drs, unsigned int a_page_count, unsigned int a_offset,  unsigned short *a_buffer, size_t a_buffer_size)
{
    size_t l_offset = 0;
    bool l_loop = true;
    for (unsigned t=0; t< a_page_count && l_loop; t++){
        size_t l_read_size;
        if (a_page_count > DRS_PAGE_COUNT_MAX){
            log_it(L_ERROR, "Too many pages %u when maximum is %u", a_page_count, DRS_PAGE_COUNT_MAX);
            return -1;
        }


        if (l_offset + DRS_PAGE_READ_SIZE <= a_buffer_size)
            l_read_size = DRS_PAGE_READ_SIZE;
        else{
            l_read_size = a_buffer_size - l_offset;
            l_loop = false;
            log_it(L_ERROR, "Page read function goes out of input buffer, size %zd is not enought, requires %zd ( page read size %zd, num pages %u",
                    a_buffer_size, a_page_count * DRS_PAGE_READ_SIZE, DRS_PAGE_READ_SIZE, a_page_count );
            return -2;
        }
        drs_read_page(a_drs,t,&a_buffer[t*a_offset], l_read_size);
    }
    return 0;
}



/**
 * unsigned int drsnum		номер drs для вычитывания сдвига
 * return 					индекс сдвига;
 */
unsigned int drs_get_shift_bank(unsigned int a_drs_num, unsigned int a_page_num)
{
    return drs_get_shift(a_drs_num, a_page_num) &1023;
}

/**
 * @brief drs_get_shift
 * @param a_drs_num
 * @return
 */
unsigned int drs_get_shift(unsigned int a_drs_num, unsigned int a_page_num)
{
    unsigned short tmpshift;
    if (a_drs_num==0)
     tmpshift=((uint32_t *)data_map_shift_drs1)[a_page_num];
    else
     tmpshift=((uint32_t *)data_map_shift_drs2)[a_page_num];
    return tmpshift & 4095;
}

int drs_data_dump_in_files_double(const char * a_filename, const double * a_data, size_t a_data_count, int a_flags)
{

    dap_string_t * l_file = dap_string_new("");

    // Добавляем путь до var/lib внутри папки приложения
    if ( a_flags & DRS_DATA_DUMP_ADD_PATH_VAR_LIB ){
        char * l_path = dap_strdup_printf("%s/var/lib",g_dap_vars.core.sys_dir);
        dap_mkdir_with_parents(l_path);
        dap_string_append_printf(l_file, "%s/", l_path);
    }

    // Добавляем собственно имя файла
    dap_string_append(l_file, a_filename);

    // Добавляем время в юникс формате
    if (a_flags & DRS_DATA_DUMP_ADD_TIMESTAMP ){
        dap_nanotime_t l_ts = dap_nanotime_now();
        //char l_ts_str[64]={0};
        //dap_nanotime_to_str(&l_ts,l_ts_str);
        dap_string_append_printf(l_file,"_%"DAP_UINT64_FORMAT_U"_",l_ts );
    }


    // Открываем CSV файл и пишем в него

    if (a_flags & DRS_DATA_DUMP_CSV){
        dap_string_t * l_fstr = dap_string_new( l_file->str);
        // Добавляем расширение
        dap_string_append(l_fstr, ".csv");
        char * l_file_str = dap_string_free(l_fstr, false);


        FILE * f = fopen(l_file_str,"w");
        for (size_t n = 0; n < a_data_count; n++){
            fprintf(f,"%lf;", a_data[n]);
        }
        fclose(f);

        DAP_DELETE(l_file_str);
    }

    // Открываем BIN файл и пишем в него
    if (a_flags & DRS_DATA_DUMP_BIN){
        dap_string_t * l_fstr = dap_string_new( l_file->str);
        // Добавляем расширение
        dap_string_append(l_fstr, ".bin");
        char * l_file_str = dap_string_free(l_fstr, false);

        FILE * f = fopen(l_file_str,"w");
        for (size_t n = 0; n < a_data_count; n++){
            fwrite(&a_data[n],sizeof (a_data[n]),1, f);
        }
        fclose(f);

        DAP_DELETE(l_file_str);
    }

    dap_string_free(l_file, true);

    return 0;
}


int drs_data_dump_in_files_unsigned(const char * a_filename, const unsigned * a_data, size_t a_data_count, int a_flags)
{

    dap_string_t * l_file = dap_string_new("");

    // Добавляем путь до var/lib внутри папки приложения
    if ( a_flags & DRS_DATA_DUMP_ADD_PATH_VAR_LIB ){
        char * l_path = dap_strdup_printf("%s/var/lib",g_dap_vars.core.sys_dir);
        dap_mkdir_with_parents(l_path);
        dap_string_append_printf(l_file, "%s/", l_path);
    }

    // Добавляем собственно имя файла
    dap_string_append(l_file, a_filename);

    // Добавляем время в юникс формате
    if (a_flags & DRS_DATA_DUMP_ADD_TIMESTAMP ){
        dap_nanotime_t l_ts = dap_nanotime_now();
        //char l_ts_str[64]={0};
        //dap_nanotime_to_str(&l_ts,l_ts_str);
        dap_string_append_printf(l_file,"_%"DAP_UINT64_FORMAT_U"_",l_ts );
    }


    // Открываем CSV файл и пишем в него

    if (a_flags & DRS_DATA_DUMP_CSV){
        dap_string_t * l_fstr = dap_string_new( l_file->str);
        // Добавляем расширение
        dap_string_append(l_fstr, ".csv");
        char * l_file_str = dap_string_free(l_fstr, false);


        FILE * f = fopen(l_file_str,"w");
        for (size_t n = 0; n < a_data_count; n++){
            fprintf(f,"%lf;", a_data[n]);
        }
        fclose(f);

        DAP_DELETE(l_file_str);
    }

    // Открываем BIN файл и пишем в него
    if (a_flags & DRS_DATA_DUMP_BIN){
        dap_string_t * l_fstr = dap_string_new( l_file->str);
        // Добавляем расширение
        dap_string_append(l_fstr, ".bin");
        char * l_file_str = dap_string_free(l_fstr, false);

        FILE * f = fopen(l_file_str,"w");
        for (size_t n = 0; n < a_data_count; n++){
            fwrite(&a_data[n],sizeof (a_data[n]),1, f);
        }
        fclose(f);

        DAP_DELETE(l_file_str);
    }

    dap_string_free(l_file, true);

    return 0;
}
