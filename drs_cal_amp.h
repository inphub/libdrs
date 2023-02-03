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

// применение калибровки дл€ €чеек
#define DRS_CAL_APPLY_Y_CELLS  BIT(0)
// межканальна€ калибровка
#define DRS_CAL_APPLY_Y_INTERCHANNEL  BIT(1)
// избавление от всплесков
#define DRS_CAL_APPLY_Y_SPLASHS  BIT(2)

// ¬ременна€ локальна€ калибровка
#define DRS_CAL_APPLY_X_TIME_LOCAL  BIT(8)
// ¬ременна€ локальна€ калибровка
#define DRS_CAL_APPLY_X_TIME_GLOBAL  BIT(9)

// приведение к физическим виличинам
#define DRS_CAL_APPLY_PHYS  BIT(31)

// “олько 9ый канал
#define DRS_CAL_APPLY_CH9_ONLY           BIT(31)



void drs_cal_y_apply(drs_t * a_drs, unsigned short *buffer,double *dBuf, int a_flags);
