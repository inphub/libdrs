/*
 * calibrate.h
 *
 *  Created on: 19 сент. 2019 г.
 *      Author: Denis
 *  Refactored: 2022 by Dmitry Gerasimov
 */
#pragma once

#include <pthread.h>
#include "drs.h"


void calibrate_do_curgr(unsigned short *buffer,double *dBuf,unsigned int *shift,coefficients_t *coef,unsigned int chanalLength,unsigned int chanalCount,unsigned int key,parameter_t *prm);

void calibrate_collect_statistics_b(double *acc,unsigned short *buff,unsigned short *shift,unsigned int *statistic);
