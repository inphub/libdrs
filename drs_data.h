/*
 * drs_proto_cmd.h
 *
 *  Created on: 1 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include <dap_common.h>
#include "drs.h"

// режим калибровки
#define DRS_OP_FLAG_CALIBRATE     BIT(0)
//индикатор внешнего запуска;
#define DRS_OP_FLAG_EXT_START     BIT(1)

#define DRS_OP_FLAG_ROTATE        BIT(3)

#define DRS_OP_FLAG_SOFT_START    BIT(4)

#define DRS_OP_FLAG_NO_CUT        BIT(5)


#define DRS_PAGE_READ_SIZE        DRS_CELLS_COUNT *sizeof(unsigned short)


#ifdef __cplusplus
extern "C" {
#endif

#define DRS_DATA_DUMP_BIN                             0x00000001
#define DRS_DATA_DUMP_CSV                             0x00000002
#define DRS_DATA_DUMP_ADD_PATH_VAR_LIB                0x00000004
#define DRS_DATA_DUMP_ADD_TIMESTAMP                   0x00000008

int drs_data_dump_in_files(const char * a_filename, const double * a_data, size_t a_data_count, int a_flags);

int drs_data_get_all(drs_t * a_drs, int a_flags , unsigned short * a_buffer); /// Если a_drs NULL то он копирует для всех DRS
int drs_data_get_page(drs_t * a_drs, int a_flags ,unsigned a_page, unsigned short * a_buffer, size_t a_buffer_size); /// Если a_drs NULL то он копирует для всех DRS
static inline int drs_data_get(drs_t * a_drs, int a_flags, unsigned short * a_buffer, size_t  a_buffer_size){
    return drs_data_get_page(a_drs, a_flags,0,a_buffer, a_buffer_size);
}

// Один квант в наносекундах
#define DRS_ZAP_DELAY_QUANT_NS       4
// Дефолтное значение
#define DRS_ZAP_DELAY_QUANT_DEFAULT  0
// Минимум
#define DRS_ZAP_DELAY_QUANT_MIN      0
// Максимум
#define DRS_ZAP_DELAY_QUANT_MAX      4096

/**
 * @brief drs_data_set_zap_delay_quants
 * @param a_drs_num
 * @param a_delay_quants
 * @return
 */
static inline int drs_data_set_zap_delay_quants(unsigned a_drs_num, unsigned a_delay_quants)
{
   drs_reg_write(DRS_REG_ZAP_DELAY_A + a_drs_num, a_delay_quants );
   return 0;
}

/**
 * @brief drs_data_set_zap_delay_ns
 * @param a_drs_num
 * @param a_delay_ns
 * @return
 */
static inline int drs_data_set_zap_delay_ns(unsigned a_drs_num, unsigned a_delay_ns){
    return drs_data_set_zap_delay_quants(a_drs_num, a_delay_ns / DRS_ZAP_DELAY_QUANT_NS );
}

/**
 * @brief drs_data_get_zap_delay_quants
 * @param a_drs_num
 */
static inline unsigned drs_data_get_zap_delay_quants(unsigned a_drs_num)
{
    if( a_drs_num >= DRS_COUNT)
        return -1;
    return drs_reg_read(DRS_REG_ZAP_DELAY_A + a_drs_num);
}

/**
 * @brief drs_data_get_zap_delay_ns
 * @param a_drs_num
 */
static inline unsigned drs_data_get_zap_delay_ns(unsigned a_drs_num){
    return drs_data_get_zap_delay_quants(a_drs_num) * DRS_ZAP_DELAY_QUANT_NS;
}


unsigned int drs_get_shift(unsigned int a_drs_num, unsigned int a_page_num);
unsigned int drs_get_shift_bank(unsigned int a_drs_num, unsigned int a_page_num);

void drs_read_page(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size);


void drs_read_page_rotated(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size);

void drs_read_pages(drs_t * a_drs, unsigned int a_page_count, unsigned int a_offset,  unsigned short *a_buffer, size_t a_buffer_size);

void drs_data_rotate_bank9(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size);
void drs_data_rotate_bank(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size);
void drs_data_rotate_global(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size);

#ifdef __cplusplus
}
#endif
