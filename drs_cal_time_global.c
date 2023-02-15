/*
 * drs_cal_time_global.c
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>
#include <math.h>

#include <dap_common.h>

#include "data_operations.h"

#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_local.h"
#include "drs_cal_time_global.h"

#define LOG_TAG  "drs_cal_time_global"


#define GeneratorFrequency 50 //MHz
#define PERIOD_LENGTH 38.912 //4.9152/2.4*19 || periodLength-������ � �������� ������ ������� 4.9152 ���-������� ���, 2400/19-��� ������� ������
#define MAX_PERIOD_COUNT 28//26,315789473684210526315789473684- ������������ ���������� �������� � 1024 ��������->27, +1 ��� ����;
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];

static bool s_debug_more = true;

static void s_proc(drs_t * a_drs, atomic_uint_fast32_t * a_progress, double *a_x, double *a_y, double *a_sum_delta_ref, double *a_stats);

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
 * @brief drs_cal_time_global_apply  ��������� ����������� ���������� � ������
 * @param a_drs                      ������ DRS
 * @param a_x                        ������ �������� ��������
 * @param a_result                  ������ � ������������
 */
void drs_cal_time_global_apply( drs_t * a_drs,  double *a_in,   double *a_out )
{
    double tmpX;
    for(unsigned b=0;b<DRS_CHANNEL_BANK_COUNT ;b++){
        tmpX=0;
//        a_in[b*DRS_CELLS_COUNT_BANK]=0;
        for(unsigned n=1, pz;n< DRS_CELLS_COUNT_BANK;n++){
            pz=( a_drs->shift + n)&1023;
            tmpX+=(a_out[n + b*DRS_CELLS_COUNT_BANK] - a_out[ (n-1) + b*DRS_CELLS_COUNT_BANK])*
                a_drs->coeffs.kTime[ pz ];
            a_in[n + b*DRS_CELLS_COUNT_BANK]=tmpX;
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
    double l_sum_delta_ref[DRS_CELLS_COUNT_BANK]={0},
        l_stats[DRS_CELLS_COUNT_BANK]={0},
        l_x[DRS_CELLS_COUNT_CHANNEL]={0}, l_y[DRS_CELLS_COUNT]={0};
    unsigned short l_y_raw[DRS_CELLS_COUNT]={0};

    // ���������� ��������
    unsigned l_progress_old = 0;
    const double l_progress_total = 30.0;
    if (a_progress)
      l_progress_old = *a_progress;

    // ��������� �� ������� ����, ��� ���� ������� ���������� ����������
    if(( a_drs->coeffs.indicator&3)!=3){// TODO �������� �� ���-�� ����� ���������
        log_it(L_WARNING,"before global timer calibration you need to do the timer calibration and amplitude calibration");
        return -1;
    }
    dap_nanotime_t l_ts_start = dap_nanotime_now();


    // ��������� ������ X
    for (unsigned n = 0; n < DRS_CELLS_COUNT_CHANNEL; n++){
        l_x[n] = n;
    }

    // �������� ����� ��������� ���������� � ��������� ������
    drs_mode_t l_mode_old = drs_get_mode(a_drs->id);
    drs_set_mode(a_drs->id, DRS_MODE_CAL_TIME);
    drs_set_sinus_signal(true);

    for( unsigned i = 0; i < a_args->param.time_global.num_cycle; i++){
        int l_ret = drs_data_get(a_drs,0, l_y_raw,sizeof (l_y_raw) );
        if ( l_ret != 0){
            log_it(L_ERROR,"data get returned with error, code %d", l_ret);
            drs_set_mode(a_drs->id, DRS_MODE_SOFT_START);
        }
        drs_cal_y_apply(a_drs, l_y_raw,l_y, DRS_CAL_APPLY_Y_CELLS |
                                                   DRS_CAL_APPLY_Y_INTERCHANNEL );
        drs_cal_time_local_apply(a_drs, l_x, l_x);
        s_proc(a_drs,  a_progress, l_x,l_y, l_sum_delta_ref,l_stats);
    }


    // ��������� ����� ������ � ������������ � ������� �����
    drs_set_sinus_signal(false);
    drs_set_mode(a_drs->id, l_mode_old);

    // ������� �����������
    for(unsigned n = 0; n < DRS_CELLS_COUNT_BANK; n++) {
        if(l_stats[n] == 0.0){
            l_stats[n] = 1.0;
        }
        if ( n < 10 ){
            debug_if(s_debug_more,L_INFO, "SumDeltaRef=%f, stats=%f", l_sum_delta_ref[n], l_stats[n]);
        }
        a_drs->coeffs.kTime[n] = l_sum_delta_ref[n] / l_stats[n];
    }

    // ��������� ����� ������ ����������
    double l_ts_diff  =  ((double) (dap_nanotime_now() - l_ts_start))/ 1000000000.0  ;
    log_it(L_NOTICE,"Finished local time calibration in %.3f seconds", l_ts_diff);

    // ��������� �������� ���
    if (a_progress)
      *a_progress = l_progress_old +l_progress_total;


    return 0;

}

/**
 * @brief s_proc
 * @param a_drs
 * @param a_progress
 * @param a_x
 * @param a_y
 * @param a_sum_delta_ref
 * @param a_stats
 */
static void s_proc(drs_t * a_drs, atomic_uint_fast32_t * a_progress, double *a_x, double *a_y, double *a_sum_delta_ref, double *a_stats)
{
    double average,
        l_last_x,l_last_y,
        period[MAX_PERIOD_COUNT]={0.0},
        periodDelt[MAX_PERIOD_COUNT] = { 0.0 }, deltX = 0.0;
    unsigned int n,pz,indexs[MAX_PERIOD_COUNT],l_count=0,l;

    // �������������� �������� ���
    double l_progress_step = 27.0 / ((double) (DRS_CHANNEL_BANK_COUNT * DRS_CELLS_COUNT_BANK)) ;
    unsigned l_progress_old = 0;
    if (a_progress)
      l_progress_old = *a_progress;
    double l_progress = 0.0;

    // ������� ������� �� y
    average = drs_ch_get_average(a_y,DRS_CELLS_COUNT_BANK,DRS_CHANNEL_9 );

    l_last_y=0;
    l_last_x=0;
    l_count=0;

    // ����� �� �������
    for(n=0;n< (DRS_CELLS_COUNT_BANK + 1);n++){
        if( (average >= l_last_y) && (a_y[n] >= average) && (n != 0) ) {
            deltX = a_x[n] - l_last_x;
            if(a_x[n] < l_last_x) {
                deltX += 1024.0;
            }

            period[l_count] = deltX / (a_y[n]-l_last_y) * (average - l_last_y) + l_last_x;
            if(l_count > 0) {
                periodDelt[l_count-1] = period[l_count] - period[l_count - 1];
            }
            indexs[l_count] = a_drs->shift + n;
            if(l_count < MAX_PERIOD_COUNT )
                l_count++;
            //else
            //    log_it(L_WARNING, "Count is on maximum ");
        }

        l_last_y = a_y[n];
        l_last_x = a_x[n];

        // ��������� �������� ���
        l_progress += l_progress_step;
        if (a_progress)
            *a_progress = l_progress_old + floor(l_progress);
    }

    // ����� �� ������������� ����������
    for(n = 1; n < l_count; n++) {
        for(l = indexs[n-1]; l < indexs[n]; l++){
            pz=l&1023;

            a_sum_delta_ref[pz]+=PERIOD_LENGTH/periodDelt[n-1];
            a_stats[pz]++;
        }
    }

}


