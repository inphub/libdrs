/*
 * drs_ops.h
 *
 *  Created on: 3 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include "drs.h"

// Разрешение программного старта записи сигнала
#define DRS_CMD_SOFT_START           0x00000001
// Загрузка всех заданных установок и запуск непосредственно самой DRS
#define DRS_CMD_LOAD_N_RUN           0x00000002
//Разрешение внешнего старта записи сигнала
#define DRS_CMD_EXT_START            0x00000004
//Разрешение внешнего старта записи сигнала
#define DRS_CMD_RESET                0x00000008

#define DRS_CMD_MAX                  0x00000008

#ifdef __cplusplus
extern "C" {
#endif

void drs_start(int a_drs_num, int a_flags, unsigned a_pages_num);
void drs_set_flag_end_read(int l_drs_num, bool a_enable);
void drs_set_num_pages_all(unsigned int a_num);
void drs_cmd(int a_drs_num, unsigned int a_cmd);

void drs_set_num_pages(drs_t * a_drs, unsigned int a_num);

bool drs_get_flag_write_ready(int l_drs_num );

void drs_set_sinus_signal(bool a_sinus_signal);
int drs_data_wait_for_ready(drs_t * a_drs);

#ifdef __cplusplus
}
#endif
