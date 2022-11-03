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


#define GeneratorFrequency 50 //MHz
#define periodLength 38.912 //4.9152/2.4*19 || periodLength-длинна в отсчетах одного периода 4.9152 √√ц-частота ацп, 2400/19-ћ√ц частота синуса
#define maxPeriodsCount 28//26,315789473684210526315789473684- максимальное количество периодов в 1024 отсчетах->27, +1 дл€ нул€;
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];
extern int g_current_freq;



typedef struct{
    int progress; // Progress between 0 and 100
    pthread_rwlock_t rwlock;
} calibrate_state_t;

void calibrate_get_array_x(double*x, unsigned int *shift,coefficients_t *coef,unsigned int key);
unsigned int calibrate_time_global(unsigned int numCycle, coefficients_t *coef,parameter_t *prm);
unsigned int calibrate_time(unsigned int minN, coefficients_t *coef,parameter_t *prm);

void calibrate_do_curgr(unsigned short *buffer,double *dBuf,unsigned int *shift,coefficients_t *coef,unsigned int chanalLength,unsigned int chanalCount,unsigned int key,parameter_t *prm);

unsigned int calibrate_amplitude(coefficients_t *coef,double *calibLvl,unsigned int N, double *shiftDAC,parameter_t *prm, unsigned int count);

int calibrate_run(int a_drs_num);
calibrate_state_t *calibrate_get_state(int a_drs_num);
void calibrate_abort(int a_drs_num);
