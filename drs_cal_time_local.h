/*
 * drs_cal_time_local.h
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#pragma once

#include "drs_cal.h"

#ifdef __cplusplus
extern "C" {
#endif

int drs_cal_time_local_init();
void drs_cal_time_local_deinit();


int drs_cal_time_local(int a_drs_num,  drs_cal_args_t* a_args,  atomic_uint_fast32_t * a_progress );
void drs_cal_time_local_apply(drs_t*  a_drs, double* a_values , double * a_output);
void drs_cal_time_local_apply_old ( double* bufferX, coefficients_t* coef, unsigned int* shift);

void drs_cal_time_local_stats_out(int a_drs_num, double * a_stats_out);

#ifdef __cplusplus
}
#endif
