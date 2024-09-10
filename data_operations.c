/*
 * data_operations.c
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: Denis
 */
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <dap_common.h>

#include "data_operations.h"
#include "commands.h"
#include "drs.h"
#include "drs_ops.h"

#define LOG_TAG "data_operations"

static bool s_debug_more = false;

/**
 * МНК
 * double*yArr			массив Y
 * double *xArr 		массив X
 * unsigned int length  длинна массивов
 * double *b			адрес переменной для записи b
 * double *k			адрес переменной для записи k
 */
void getCoefLine(double*yArr,double *xArr,unsigned int length,double *b,double *k)
{
    // length может быть 0, тогда массивы yArr и xArr будут NULL
    double x=0,x2=0,xy=0,y=0;
    unsigned int i=0;
    for(i=0;i<length;i++){
            x+=xArr[i];
            y+=yArr[i];
            xy+=xArr[i]*yArr[i];
            x2+=xArr[i]*xArr[i];
    }
    *k= ( ((double) length)*xy- x*y )/(  ((double)length)*x2 - x*x);
    *b=  (y-*k*x)/ ((double)length);
    //printf("k=%f\tb=%f\n",*k,*b);
}
void do_on_array(double *array,int length,void (*operation)(double *num)){
	int i;
	for(i=0;i<length;i++){
		(*operation)(&array[i]);
	}
}


/**
 * double *average					масиив со средними значениями каналов;
 * double *data						массив данных
 * unsigned int chanalLength		длинна канала;
 * unsigned int chanalCount			число каналов;
 */
void getAverage(double *average,double *data,unsigned int chanalLength,unsigned int chanalCount)
{
    unsigned int i,j;
	for(i=0;i<chanalCount;i++)
	{
		average[i]=0.0;
		for(j=0;j<chanalLength;j++)
		{
			average[i]+=data[j*chanalCount+i];
		}
		average[i]/= (double) chanalLength;
		//printf("average[%d]=%f\n",i,average[i]);
	}
}

/**
 * @brief drs_ch_get_average
 * @param a_cells
 * @param a_cells_count
 * @param a_ch_id
 * @return
 */
double drs_ch_get_average(double *a_cells, unsigned a_cells_count, unsigned a_ch_id)
{
    double l_acc=0.0;
    double l_min=0.0, l_max=0.0;
    unsigned l_cells_count = a_cells_count - g_drs_data_cut_from_end - g_drs_data_cut_from_begin;
    for(unsigned n=g_drs_data_cut_from_begin; n< (a_cells_count - g_drs_data_cut_from_end) ; n++){
        unsigned l_idx = n* DRS_CHANNELS_COUNT + a_ch_id;
        double l_cell = a_cells[l_idx];
        l_acc += l_cell;
        if(l_cell < l_min)
            l_min = l_cell;
        if(l_cell > l_max)
            l_max = l_cell;
    }

    double l_ret = l_acc / ((double) (l_cells_count ));

    debug_if(s_debug_more,L_DEBUG,"drs_ch_get_average: l_acc=%f, l_ret=%f, l_min=%f, l_max=%f, l_cells_count=%u",
             l_acc, l_ret, l_min, l_max, l_cells_count);

    return l_ret;
}




