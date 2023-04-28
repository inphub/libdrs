/*
 * calibrate.c
 *
 *  Created on: 19 сент. 2019 г.
 *      Author: Denis
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <dap_common.h>

#include "calibrate.h"
#include "commands.h"
#include "data_operations.h"

#include "drs.h"
#include "drs_cal.h"
#include "drs_ops.h"
#include "drs_data.h"


#define LOG_TAG "calibrate"

#define GeneratorFrequency 50 //MHz
#define periodLength 38.912 //4.9152/2.4*19 || periodLength-длинна в отсчетах одного периода 4.9152 ГГц-частота ацп, 2400/19-МГц частота синуса
#define maxPeriodsCount 28//26,315789473684210526315789473684- максимальное количество периодов в 1024 отсчетах->27, +1 для нуля;
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];

static void           s_remove_splash     (   double* a_Y,             unsigned int* shift,       coefficients_t* coef     );






/**
 * @brief s_remove_splash
 * @param a_Y
 * @param shift
 * @param coef
 */
static void           s_remove_splash     (   double* a_Y,             unsigned int* shift,       coefficients_t* coef     )
{
    size_t i=0,j=0;
    for(j=0;j<DRS_CHANNELS_COUNT;j++){
        i = ( coef->splash[j] - shift[0] + 1023) & 1023;
        if((i > 0) && (i < 1023)){
            a_Y[i*4+j] = (a_Y[(i + 1) * 4 + j] + a_Y[(i - 1) * 4 + j]) / 2;
        }

        if(i == 0){
            a_Y[j] = (a_Y [4*1 + j] + a_Y[4*2 + j]) / 2;
        }

        if(i == 1023){
            a_Y[1023 * 4 + j]=(a_Y[j] + a_Y[1022 * 4 + j]) / 2;
        }
    }
}



/**
 * Собирает статистику для каждой ячкейки
 * coefficients *acc			сумма знчаений для ячейки;
 * unsigned short *buff 		массив данных;
 * unsigned int shift			индекс разварота данных;
 * unsigned int *statistic		статистика для ячеек;
 */
void calibrate_collect_statistics_b(double *acc,unsigned short *buff,unsigned short *shift,unsigned int *statistic)
{
	unsigned int j,k,rotateIndex;
		for(j=0;j<4;j++)
		{
			for(k=0;k<1000;k++)
			{
				rotateIndex=(shift[j>>1]+k)&1023;
				acc[rotateIndex*4+j]+=buff[k*4+j];
				statistic[rotateIndex*4+j]++;
			}
		}
}



