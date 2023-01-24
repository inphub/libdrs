/*
 * drs_cal_time_global.c
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>
#include <dap_common.h>

#include "data_operations.h"

#include "drs.h"
#include "drs_data.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_local.h"
#include "drs_cal_time_global.h"

#define LOG_TAG  "drs_cal_time_global"


#define GeneratorFrequency 50 //MHz
#define periodLength 38.912 //4.9152/2.4*19 || periodLength-длинна в отсчетах одного периода 4.9152 ГГц-частота ацп, 2400/19-МГц частота синуса
#define maxPeriodsCount 28//26,315789473684210526315789473684- максимальное количество периодов в 1024 отсчетах->27, +1 для нуля;
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];

static void s_proc(drs_t * a_drs, double *x, double *y, double *sumDeltaRef, double *statistic);

static int s_proc_drs(drs_t * a_drs, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);


/**
 * @brief drs_cal_time_global
 * @param a_drs_num
 * @param a_args
 * @param a_progress
 * @return
 */
int drs_cal_time_global( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress)
{
    if (a_drs_num == -1){
        int l_ret = -10;
        for (size_t i=0; i < DRS_COUNT; i++ ){
            l_ret = s_proc_drs(g_drs +i, a_args,a_progress);
            if ( l_ret != 0){
                log_it(L_ERROR,"Can't run calibrate amplitude for DRS #%u code %d ",i, l_ret);
                break;
            }
        }
        return l_ret;
    }else if (a_drs_num < DRS_COUNT){
        int l_ret = s_proc_drs(g_drs + a_drs_num, a_args,a_progress);
        if ( l_ret != 0){
            log_it(L_ERROR,"Can't run calibrate amplitude for DRS #%u code %d ",a_drs_num, l_ret);
        }
        return l_ret;
    }else{
        log_it(L_WARNING, "Wrong DRS number %u, can't be more than %u", a_drs_num, DRS_COUNT);
        return -11;
    }
}


/**
 * @brief drs_cal_time_global_apply  Применяет амплитудную калибровку к данным
 * @param a_drs                      Объект DRS
 * @param a_x                        Массив входящих значений
 * @param a_result                  Массив с результатами
 */
void drs_cal_time_global_apply( drs_t * a_drs,  double *a_x,   double *a_result )
{
    unsigned int i,j,pz;
    double tmpX;
    for(i=0;i<DRS_CHANNELS_COUNT;i++)
    {
        a_x[i]=0;
        tmpX=0;
        for(j=1;j<1024;j++)
        {
            pz=( a_drs->shift+j)&1023;
            tmpX+=(a_result[j*DRS_CHANNELS_COUNT+i] - a_result[(j-1)*DRS_CHANNELS_COUNT+i])*
                a_drs->coeffs.kTime[pz*DRS_CHANNELS_COUNT+i];
            a_x[j*4+i]=tmpX;
        }
    }
}


/**
 * double *x			указатель на массив для значений x;
 * double *xMas;				рузельтат applyTimeCalibration;
 * unsigned int shift		сдвиг в данных;
 * coefficients *coef		структура с коэффициентами;
 */
void drs_cal_time_global_apply_old(double *x,double *xMas, unsigned int *shift, coefficients_t *coef)
{
    unsigned int i,j,pz;
    double tmpX;
    for(i=0;i<4;i++)
    {
        x[i]=0;
        tmpX=0;
        for(j=1;j<1024;j++)
        {
            pz=(shift[i>>1]+j)&1023;
            tmpX+=(xMas[j*4+i] - xMas[(j-1)*4+i])*coef->kTime[pz*4+i];
            x[j*4+i]=tmpX;
        }
    }
}


/**
 * @brief s_proc_drs
 * @param a_drs
 * @param a_args
 * @param a_progress
 * @return
 */
static int s_proc_drs(drs_t * a_drs, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress)
{
    assert(a_drs);
    assert(a_args);
    double sumDeltaRef[2048]={0}, statistic[32768]={0},x[32768]={0},dBuf[32768]={0};
    unsigned short pageBuffer[DRS_PAGE_READ_SIZE /sizeof (unsigned short)],ind = 0;
    unsigned int k,t;
    //unsigned char i=0;

    if(( a_drs->coeffs.indicator&3)!=3)
    {
        log_it(L_WARNING,"before global timer calibration you need to do the timer calibration and amplitude calibration");
        return 0;
    }
    drs_set_mode(a_drs->id, DRS_MODE_CAL_TIME);
    while(ind< a_args->param.time_global.num_cycle)
    {
        ind++;
        int l_ret = drs_data_get(a_drs,0, pageBuffer,sizeof (pageBuffer) );
        if ( l_ret != 0){
            log_it(L_ERROR,"data get returned with error, code %d", l_ret);
            drs_set_mode(a_drs->id, DRS_MODE_SOFT_START);
        }
        drs_cal_ampl_apply(a_drs, pageBuffer,dBuf, DRS_CAL_AMPL_APPLY_CELLS |
                                                   DRS_CAL_AMPL_APPLY_INTERCHANNEL );
        drs_cal_time_local_apply(a_drs, x, x);
        s_proc(a_drs, x,dBuf, sumDeltaRef,statistic);
    }
    drs_set_mode(a_drs->id, DRS_MODE_SOFT_START);

    for(k=0;k<4;k++)
    {
        for(t=0;t<1024;t++)
        {
            if(statistic[t*4+k]==0)
            {
                statistic[t*4+k]=1;
            }
            a_drs->coeffs.kTime[t*4+k]=sumDeltaRef[t*4+k]/statistic[t*4+k];
        }
    }
    return 1;

}
/**
 * @brief s_proc
 * @param a_drs
 * @param x
 * @param y
 * @param sumDeltaRef
 * @param statistic
 */
static void s_proc(drs_t * a_drs, double *x, double *y, double *sumDeltaRef, double *statistic)
{
    double average[4],lastX,lastY,period[maxPeriodsCount],periodDelt[maxPeriodsCount],deltX;
    unsigned int i,j,pz,indexs[maxPeriodsCount],count=0,l;
    getAverage(average,y,1000,4);
    for(i=0;i<4;i++)
    {
        lastY=0;
        lastX=0;
        count=0;
        for(j=0;j<1001;j++)
        {
            if((average[i]>=lastY)&&(y[j*4+i]>=average[i])&&(j!=0))
            {
                deltX=x[j*4+i]-lastX;
                if(x[j*4+i]<lastX)
                {
                    deltX+=1024;
                }
                period[count]=deltX/(y[j*4+i]-lastY)*(average[i]-lastY)+lastX;
                if(count>0)
                {
                    periodDelt[count-1]=period[count]-period[count-1];
                }
                indexs[count]= a_drs->shift+j;
                count++;
            }
            lastY=y[j*4+i];
            lastX=x[j*4+i];
        }
        for(j=1;j<count;j++)
        {
            for(l=indexs[j-1];l<indexs[j];l++)
            {
                pz=l&1023;

                sumDeltaRef[pz*4+i]+=periodLength/periodDelt[j-1];
                statistic[pz*4+i]++;
            }
        }
    }

}


