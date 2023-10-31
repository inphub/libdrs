/*
 * drs_cal_amp.h
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#pragma once
#include <dap_common.h>
#include "drs_cal.h"
#include "drs_cal_pvt.h"

#ifdef __cplusplus
extern "C" {
#endif

int drs_cal_amp( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);
void drs_cal_amp_remove_splash(drs_t * a_drs, double*a_Y, double a_treshold, int a_flags);


#ifdef __cplusplus
}
#endif
