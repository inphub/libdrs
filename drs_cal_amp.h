/*
 * drs_cal_amp.h
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#pragma once
#include <dap_common.h>
#include "drs_cal.h"

int drs_cal_amp( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);

// применение калибровки для ячеек
#define DRS_CAL_AMPL_APPLY_CELLS  BIT(0)
// межканальная калибровка
#define DRS_CAL_AMPL_APPLY_INTERCHANNEL  BIT(1)
// избавление от всплесков
#define DRS_CAL_AMPL_APPLY_SPLASHS  BIT(2)
// приведение к физическим виличинам
#define DRS_CAL_AMPL_APPLY_PHYS  BIT(3)


void drs_cal_ampl_apply(drs_t * a_drs, unsigned short *buffer,double *dBuf, int a_flags);
