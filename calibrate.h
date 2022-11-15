/*
 * calibrate.h
 *
 *  Created on: 19 ����. 2019 �.
 *      Author: Denis
 *  Refactored: 2022 by Dmitry Gerasimov
 */
#pragma once

#include <pthread.h>
#include "drs.h"


#define GeneratorFrequency 50 //MHz
#define periodLength 38.912 //4.9152/2.4*19 || periodLength-������ � �������� ������ ������� 4.9152 ���-������� ���, 2400/19-��� ������� ������
#define maxPeriodsCount 28//26,315789473684210526315789473684- ������������ ���������� �������� � 1024 ��������->27, +1 ��� ����;
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];
extern int g_current_freq;


void calibrate_get_array_x(double*x, unsigned int *shift,coefficients_t *coef,unsigned int key);
unsigned int calibrate_time_global(unsigned int numCycle, coefficients_t *coef,parameter_t *prm);
unsigned int calibrate_time(unsigned int minN, coefficients_t *coef,parameter_t *prm);

void calibrate_do_curgr(unsigned short *buffer,double *dBuf,unsigned int *shift,coefficients_t *coef,unsigned int chanalLength,unsigned int chanalCount,unsigned int key,parameter_t *prm);

unsigned int calibrate_amplitude(coefficients_t *coef,double *calibLvl,unsigned int N, parameter_t *prm, unsigned int count, atomic_uint_fast32_t * a_progress);

void calibrate_collect_statistics_b(double *acc,unsigned short *buff,unsigned short *shift,unsigned int *statistic);
