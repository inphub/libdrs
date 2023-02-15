/*
 * drs_cal_time_global.h
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#pragma once

#include "drs_cal.h"
int drs_cal_time_global( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);

void drs_cal_time_global_apply( drs_t * a_drs,  double *a_in,   double *a_out );

