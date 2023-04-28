/*
 * data_operations.h
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: Denis
 */
#pragma once

#include <dap_common.h>
#include "drs.h"

void getCoefLine(double*yArr,double *xArr,unsigned int length,double *b,double *k);
void do_on_array(double *array,int length,void (*operation)(double *num));
void pageModeStart(unsigned int num);
unsigned int onceGet(unsigned short *buffer,unsigned int *shift,unsigned int calibrate,unsigned int extStart,unsigned int drsnum);
unsigned int getShiftIndex(unsigned int drsnum);
double absf(double value);

/**
 * @brief fill_array
 * @param a_array
 * @param a_value
 * @param a_array_length
 * @param a_value_size
 */
static inline void fill_array(void *a_array,const void *a_value,size_t a_array_length, size_t a_value_size)
{
    for(size_t i=0;i<a_array_length;i++){
        memcpy( ((byte_t*) a_array)+ i*a_value_size,a_value,a_value_size);
    }
}

void getAverage(double *average,double *data,unsigned int chanalLength,unsigned int chanalCount);
void getAverageInt(double *average,unsigned short *data,unsigned int chanalLength,unsigned int chanalCount);
double drs_ch_get_average(double *a_cells, unsigned a_cells_count, unsigned a_ch_id);

void readNPage(unsigned short *buffer,unsigned int *shift,unsigned int numPage, unsigned int drsnum);
void writeNPage(unsigned short *buffer,unsigned int numPage, unsigned int drsnum);
void readNPages(unsigned short *buffer,unsigned int *shift,unsigned int pageCount, unsigned int step, unsigned int drsnum);
