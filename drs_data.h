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

#define DRS_PAGE_READ_SIZE        DRS_CELLS_COUNT *sizeof(unsigned short)


#ifdef __cplusplus
extern "C" {
#endif

int drs_data_get(drs_t * a_drs, int a_flags, unsigned short * a_buffer, size_t  a_buffer_size);
int drs_data_get_all(drs_t * a_drs, int a_flags , unsigned short * a_buffer); /// Если a_drs NULL то он копирует для всех DRS

unsigned int drs_get_shift(unsigned int a_drs_num);
unsigned int drs_get_shift_bank(unsigned int a_drs_num);


void drs_read_page(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size);
void drs_read_page_rotated(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer, size_t a_buffer_size);
static inline void drs_read_page_all(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer)
{
    return drs_read_page(a_drs, a_page_num, a_buffer, DRS_PAGE_READ_SIZE);
}

void drs_read_pages(drs_t * a_drs, unsigned int a_page_count, unsigned int a_step,  unsigned short *a_buffer, size_t a_buffer_size);

void drs_data_rotate(drs_t * a_drs, const void * a_mem_in, void * a_mem_out, size_t a_mem_size, const size_t a_cell_size);

#ifdef __cplusplus
}
#endif
