/*
 * drs_cal_time_local.c
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>

#include "data_operations.h"

#include "drs.h"
#include "drs_ops.h"
#include "drs_data.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_local.h"

#define LOG_TAG "drs_cal_time_local"

static double s_get_deltas_min (double*               a_buffer,  double*          a_sum_delta_ref,
                                double*               a_stats,   unsigned int     a_shift);
static int    s_proc_drs       (drs_t*                a_drs,     drs_cal_args_t*  a_args,
                                atomic_uint_fast32_t* a_progress);

/**
 * @brief drs_cal_time_local
 * @param a_drs_num
 * @param a_args
 * @param a_progress
 * @return
 */
int drs_cal_time_local( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress)
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
 * @brief drs_cal_time_local_apply
 * @param a_drs
 * @param a_values
 */
void drs_cal_time_local_apply(drs_t * a_drs, double * a_values )
{
    assert(a_drs);
    unsigned int k,t,pz;
    double l_average[4],l_tmp_value;
    for(k=0;k<4;k++)
    {
        l_average[k]=0;
        for(t=0;t<1024;t++)
        {
            l_average[k]+= a_drs->coeffs.deltaTimeRef[t*4+k];
        }
        l_average[k]=1023/l_average[k];
    }
    for(k=0;k<4;k++)
    {
        l_tmp_value=0;
        for(t=0;t<1024;t++)
        {
            pz=( a_drs->shift+t)&1023;
            a_values[t*4+k]=l_tmp_value;
            l_tmp_value+= a_drs->coeffs.deltaTimeRef[pz*4+k]*l_average[k];
        }
    }
}


/**
 * @brief drs_cal_time_local_apply
 * @param x
 * @param coef
 * @param shift
 */
void drs_cal_time_local_apply_old( double*x, coefficients_t *coef,unsigned int *shift)
{
    unsigned int k,t,pz;
    double average[4],tmpX;
    for(k=0;k<4;k++)
    {
        average[k]=0;
        for(t=0;t<1024;t++)
        {
            average[k]+=coef->deltaTimeRef[t*4+k];
        }
        average[k]=1023/average[k];
    }
    for(k=0;k<4;k++)
    {
        tmpX=0;
        for(t=0;t<1024;t++)
        {
            pz=(shift[k>>1]+t)&1023;
            x[t*4+k]=tmpX;
            tmpX+=coef->deltaTimeRef[pz*4+k]*average[k];
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
    int l_ret = 0;
    double minValue=0,sumDeltaRef[8192], statistic[8192],dBuf[8192],dn=0;
    unsigned short *pageBuffer = DAP_NEW_Z_SIZE(unsigned short, sizeof(unsigned short)*DRS_PAGE_READ_SIZE);

    unsigned int k=0,t,n=0;
    unsigned minN = a_args->param.time_local.min_N;
    coefficients_t * coef = &a_drs->coeffs;


    fill_array((unsigned char*)(statistic),(unsigned char*)&dn,8192,sizeof(dn));
    fill_array((unsigned char*)(sumDeltaRef),(unsigned char*)&dn,8192,sizeof(dn));
    if((coef->indicator&1)!=1){
            log_it(L_WARNING, "before timer calibration you need to do the amplitude calibration");
            l_ret = -1;
            goto lb_exit;
    }

    drs_mode_set(4);

    while(minValue < minN) {
        k++;
        n++;
        log_it(L_DEBUG, "prod drs #%u, min=%u\tminValue=%f",a_drs->id, minN,minValue);
        int l_ret = drs_data_get_all(NULL,0, pageBuffer);
        if(l_ret!=0){
            log_it(L_ERROR,"data_get_all not read");
            drs_mode_set(0);
            return 0;
        }
        drs_cal_ampl_apply(a_drs, pageBuffer, dBuf,DRS_CAL_AMPL_APPLY_CELLS |
                                                   DRS_CAL_AMPL_APPLY_INTERCHANNEL);
        minValue=s_get_deltas_min(dBuf,sumDeltaRef,statistic,a_drs->shift);

    }
    for(k=0;k<4;k++)
    {
        for(t=0;t<1024;t++)
        {
            coef->deltaTimeRef[t*4+k]=sumDeltaRef[t*4+k]/statistic[t*4+k];
        }
    }
    drs_mode_set(0);

    coef->indicator|=2;
lb_exit:
    DAP_DELETE(pageBuffer);
    return l_ret;
}

/**
 * double *buffer			массив с данными;
 * double *sumDeltaRef			массив дельт;
 * double *statistic			массив статистики по дельтам;
 * unsigned int shift			двиг в данных;
 */
static double s_get_deltas_min(double*a_buffer,double *a_sum_delta_ref,double *a_stats,unsigned int a_shift)
{
    unsigned int i,j, pz, pz1;
    double l_average[4];
    double l_vmin,l_vmax, l_vtmp,l_min;
    getAverage(l_average,a_buffer,1000,4);
    for(i=0;i<4;i++)
    {
        if ((16383-l_average[i])>l_average[i])
        {
            l_vtmp=l_average[i]/2;
        }else{
            l_vtmp=(16383-l_average[i])/2;
        }
        l_vmin=l_average[i]-l_vtmp;
        l_vmax=l_average[i]+l_vtmp;
        for(j=0;j<1024;j++)
        {
            pz=(a_shift+j)&1023;
            pz1=(j+1)&1023;
            if ((a_buffer[j*4+i]<=l_vmax) && (a_buffer[pz1*4+i]>=l_vmin) )
            {
                a_sum_delta_ref[pz*4+i]+=absf(a_buffer[j*4+i]-a_buffer[pz1*4+i]);
                a_stats[pz*4+i]++;
            }
        }
    }
    l_min=a_stats[0];
    for(i=0;i<4;i++)
    {
        for(j=0;j<1024;j++)
        {
            if(a_stats[j*4+i]<l_min)
            {
                l_min=a_stats[j*4+i];
            }
        }
    }
    return l_min;
}
