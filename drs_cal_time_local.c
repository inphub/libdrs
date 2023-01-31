/*
 * drs_cal_time_local.c
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>
#include <math.h>

#include "data_operations.h"

#include "drs.h"
#include "drs_ops.h"
#include "drs_data.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_local.h"

#define LOG_TAG "drs_cal_time_local"


bool s_debug_more = false;

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
void drs_cal_time_local_apply(drs_t * a_drs, double * a_values, double * a_output )
{
    assert(a_drs);
    unsigned int l_cell_id_shifted;
    bool l_is_9_channel = drs_get_mode(a_drs->id) == DRS_MODE_CAL_TIME;
    double l_average[DRS_CHANNELS_COUNT];
    unsigned l_cells_proc_count = l_is_9_channel? DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL;
    for(unsigned ch=0; ch<DRS_CHANNELS_COUNT; ch++){
        if (l_is_9_channel)
            ch = DRS_CHANNEL_9;

        l_average[ch]=0;
        for(unsigned n=0;n<l_cells_proc_count;n++){
            l_average[ch]+= a_drs->coeffs.deltaTimeRef[ n% DRS_CELLS_COUNT_BANK ] ;
        }
        l_average[ch] = ((double) (l_cells_proc_count-1 )) / l_average[ch];

        if (l_is_9_channel)
            break;
    }

    for(unsigned ch=0;ch<DRS_CHANNELS_COUNT;ch++) {
        double l_tmpX = 0;
        if (l_is_9_channel)
            ch = DRS_CHANNEL_9;

        for(unsigned b=0;b <  (l_is_9_channel? 1: DRS_CHANNELS_BANK_COUNT) ;b++) {
            for( unsigned n=0; n<l_cells_proc_count; n++) {
                l_cell_id_shifted= b* DRS_CELLS_COUNT_BANK + ( a_drs->shift+n)&1023;
                a_output[DRS_IDX(ch,n + b* DRS_CELLS_COUNT_BANK)] = l_tmpX;

                l_tmpX += a_drs->coeffs.deltaTimeRef[l_cell_id_shifted] * l_average[ch];
                //a_output[DRS_IDX(ch,n)] =  a_values[DRS_IDX(ch,n)] +
                //                        a_drs->coeffs.deltaTimeRef[l_cell_id_shifted% DRS_CELLS_COUNT_BANK] * l_average[ch];
            }
        }
        if (l_is_9_channel)
            break;
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
    int l_rc = 0;
    double l_value_min=0;

    double *l_sum_delta_ref =  DAP_NEW_Z_SIZE(double, sizeof(double)*DRS_CELLS_COUNT_BANK);
    double *l_stats =  DAP_NEW_Z_SIZE(double, sizeof(double)*DRS_CELLS_COUNT_BANK);

    double *l_cells = DAP_NEW_Z_SIZE(double, sizeof(double)*DRS_CELLS_COUNT);
    unsigned short *l_page_buffer = DAP_NEW_Z_SIZE(unsigned short, sizeof(unsigned short)*DRS_CELLS_COUNT);
    assert(l_sum_delta_ref && l_stats && l_cells && l_page_buffer);

    unsigned l_N_min = a_args->param.time_local.min_N;
    coefficients_t * coef = &a_drs->coeffs;

    unsigned l_progress_old = 0;
    if (a_progress)
      l_progress_old = *a_progress;

    if((coef->indicator&1)!=1){
            log_it(L_WARNING, "before timer calibration you need to do the amplitude calibration");
            l_rc = -1;
            goto lb_exit;
    }

    drs_set_mode(a_drs->id, DRS_MODE_CAL_TIME);
    drs_set_sinus_signal(true);

    log_it(L_NOTICE, "Process time calibration for DRS #%u with maximum repeats %u and minumum N %u", a_drs->id,
           a_args->param.time_local.max_repeats,
                    a_args->param.time_local.min_N);

    dap_nanotime_t l_ts_start = dap_nanotime_now();

    const double l_progress_total = 30.0;


    double l_progress = 0, l_progress_step = l_progress_total / a_args->param.time_local.max_repeats;

    for(unsigned n=0; l_value_min < l_N_min && n < a_args->param.time_local.max_repeats; n++, l_progress += l_progress_step ) {
        if (a_progress)
            *a_progress = l_progress_old + floor(l_progress);

        debug_if(s_debug_more, L_INFO, "prod drs #%u, min=%u\tminValue=%f",a_drs->id, l_N_min,l_value_min);
        int l_ret = drs_data_get_all(a_drs,0, l_page_buffer);
        if(l_ret!=0){
            log_it(L_ERROR,"data_get_all not read");
            drs_set_mode(a_drs->id, DRS_MODE_SOFT_START);
            l_rc = -2;
            goto lb_exit;
        }

        drs_cal_ampl_apply(a_drs, l_page_buffer, l_cells,DRS_CAL_AMPL_APPLY_CELLS | DRS_CAL_AMPL_CH9_ONLY);


        l_value_min=s_get_deltas_min(l_cells,l_sum_delta_ref,l_stats,a_drs->shift);
        debug_if(s_debug_more, L_DEBUG, "l_value_min=%f",l_value_min);
        debug_if(s_debug_more, L_DEBUG, "l_page_buffer[]={0x%04X,0x%04X,0x%04X,0x%04X...}", l_page_buffer[0],
            l_page_buffer[1], l_page_buffer[2], l_page_buffer[3]);
        debug_if(s_debug_more, L_DEBUG, "l_cells[]={%f,%f,%f,%f...}", l_cells[0], l_cells[1], l_cells[2], l_cells[3]);

    }
    double l_ts_diff  =  ((double) (dap_nanotime_now() - l_ts_start))/ 1000000000.0  ;
    log_it(L_NOTICE,"Finished local time calibration in %.3f seconds", l_ts_diff);
    for(unsigned n=0; n<DRS_CELLS_COUNT_BANK; n++){
        if(l_stats[n])
            coef->deltaTimeRef[n]=l_sum_delta_ref[n]/l_stats[n];
        else{
            log_it(L_WARNING, "Zero l_stats[n:%u]=%.3f ( l_sum_delta_ref[n]=%.3f )", n, l_stats[n], l_sum_delta_ref[n]);
            coef->deltaTimeRef[n] = 0.0;
        }
    }
    drs_set_mode(a_drs->id, DRS_MODE_SOFT_START);
    drs_set_sinus_signal(false);

    coef->indicator|=2;
lb_exit:
    DAP_DELETE( l_page_buffer );
    DAP_DELETE( l_sum_delta_ref );
    DAP_DELETE( l_stats );
    DAP_DELETE( l_cells );
    if (a_progress)
      *a_progress = l_progress_old +l_progress_total;

    return l_rc;
}

/**
 * double *buffer			массив с данными;
 * double *sumDeltaRef			массив дельт;
 * double *statistic			массив статистики по дельтам;
 * unsigned int shift			двиг в данных;
 */
static double s_get_deltas_min(double*a_buffer,double *a_sum_delta_ref,double *a_stats,unsigned int a_shift)
{
    unsigned int n, pz, pz1;
    double l_vmin,l_vmax, l_vtmp,l_min;
    double l_average = drs_ch_get_average(a_buffer,DRS_CELLS_COUNT_BANK, DRS_CHANNEL_9);

    double l_average_inverted = DRS_ADC_TOP_LEVEL-l_average - 1.0;
    debug_if(s_debug_more,L_INFO,"l_average=%f, l_average_inverted=%f", l_average, l_average_inverted);


    if ((l_average_inverted) > l_average) {
        l_vtmp = l_average/3.0;
    }else{
        l_vtmp=( l_average_inverted)/3.0;
    }
    l_vmin=l_average-l_vtmp;
    l_vmax=l_average+l_vtmp;

    for(n=0; n < DRS_CELLS_COUNT_BANK ;n++) {
        pz=(a_shift+n)&1023;
        pz1=(n+1)&1023;
        double l_cell_n = a_buffer[DRS_IDX_CAL(n)];
        double l_cell_pz1 =  a_buffer[DRS_IDX_CAL(pz1)];
        if (( (l_cell_n <=l_vmax)   &&  (l_cell_n >= l_vmin  ) )&&
            ( (l_cell_pz1 <=l_vmax) &&  (l_cell_pz1 >= l_vmin) ) &&
            ( l_cell_pz1 > l_cell_n )){

            a_sum_delta_ref[pz]+=absf( l_cell_n - l_cell_pz1 );
            a_stats[pz]++;
            //log_it(L_INFO, "a_stats[%u]=%f", pz, a_stats[pz]);
        }
    }

    l_min=a_stats[0];
    for(n=0;n< DRS_CELLS_COUNT_BANK;n++) {
        if(a_stats[n] < l_min) {
            l_min = a_stats[n];
        }
    }
    return l_min;
}
