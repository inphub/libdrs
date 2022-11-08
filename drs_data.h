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

#define DRS_PAGE_READ_SIZE        0x8000

int drs_data_get(drs_t * a_drs, unsigned short * a_buffer, int a_flags );
void drs_read_page(drs_t * a_drs,unsigned int a_page_num,  unsigned short *a_buffer);
