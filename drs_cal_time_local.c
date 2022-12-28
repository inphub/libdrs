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


bool s_debug_more = true;

static double s_get_deltas_min (double*               a_buffer,  double**         a_sum_delta_ref,
                                double**              a_stats,   unsigned int     a_shift);
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
    unsigned int pz;
    double l_average[4],l_tmp_value;
    for(unsigned ch=0; ch<DRS_CHANNELS_COUNT; ch++)
    {
        l_average[ch]=0;
        for(unsigned n=0;n<DRS_CELLS_COUNT_CHANNEL;n++){
            l_average[ch]+= a_drs->coeffs.deltaTimeRef[ch][n];
        }
        l_average[ch]=1023/l_average[ch];
    }
    for(unsigned ch=0;ch<DRS_CHANNELS_COUNT;ch++) {
        l_tmp_value=0;
        for( unsigned n=0; n<DRS_CELLS_COUNT_CHANNEL; n++) {
            pz=( a_drs->shift+n)&1023;
            a_values[n*DRS_CHANNELS_COUNT+ch]=l_tmp_value;
            l_tmp_value+= a_drs->coeffs.deltaTimeRef[ch][pz]*l_average[ch];
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
    int l_rc = 0;
    double l_value_min=0;

    double **l_sum_delta_ref = DAP_NEW_Z_SIZE(double *, sizeof (double*) * DRS_CHANNELS_COUNT);
    for (size_t ch = 0; ch <  DRS_CHANNELS_COUNT; ch ++)
         l_sum_delta_ref[ch] = DAP_NEW_Z_SIZE(double, sizeof(double)*DRS_CELLS_COUNT_CHANNEL);

    double **l_stats = DAP_NEW_Z_SIZE(double *, sizeof (double*) * DRS_CHANNELS_COUNT);
    for (size_t ch = 0; ch <  DRS_CHANNELS_COUNT; ch ++)
         l_stats[ch] = DAP_NEW_Z_SIZE(double, sizeof(double)*DRS_CELLS_COUNT_CHANNEL);

    double *l_cells = DAP_NEW_Z_SIZE(double, sizeof(double)*DRS_CELLS_COUNT);
    unsigned short *l_page_buffer = DAP_NEW_Z_SIZE(unsigned short, sizeof(unsigned short)*DRS_CELLS_COUNT);
    assert(l_sum_delta_ref && l_stats && l_cells && l_page_buffer);

    unsigned l_N_min = a_args->param.time_local.min_N;
    coefficients_t * coef = &a_drs->coeffs;

    if((coef->indicator&1)!=1){
            log_it(L_WARNING, "before timer calibration you need to do the amplitude calibration");
            l_rc = -1;
            goto lb_exit;
    }

    drs_set_mode(a_drs->id, MODE_CAL_TIME);

    static const unsigned c_repeats_max = 1000000;
    for(unsigned n=0; l_value_min < l_N_min && n < c_repeats_max; n++ ) {
        log_it(L_DEBUG, "prod drs #%u, min=%u\tminValue=%f",a_drs->id, l_N_min,l_value_min);
        int l_ret = drs_data_get_all(a_drs,0, l_page_buffer);
        if(l_ret!=0){
            log_it(L_ERROR,"data_get_all not read");
            drs_set_mode(a_drs->id, MODE_SOFT_START);
            l_rc = -2;
            goto lb_exit;
        }
        debug_if(s_debug_more, L_DEBUG, "l_page_buffer[]={0x%04X,0x%04X,0x%04X,0x%04X...}", l_page_buffer[0],
            l_page_buffer[1], l_page_buffer[2], l_page_buffer[3]);

        drs_cal_ampl_apply(a_drs, l_page_buffer, l_cells,DRS_CAL_AMPL_APPLY_CELLS );

        debug_if(s_debug_more, L_DEBUG, "l_cells[]={%f,%f,%f,%f...}", l_cells[0], l_cells[1], l_cells[2], l_cells[3]);

        l_value_min=s_get_deltas_min(l_cells,l_sum_delta_ref,l_stats,a_drs->shift);

    }
    for(unsigned ch=0;ch<DRS_CHANNELS_COUNT;ch++) {
        for(unsigned n=0;n<DRS_CELLS_COUNT_CHANNEL;n++)
        {
            coef->deltaTimeRef[ch][n]=l_sum_delta_ref[ch][n]/l_stats[ch][n];
        }
    }
    drs_set_mode(a_drs->id, MODE_SOFT_START);

    coef->indicator|=2;
lb_exit:
    DAP_DELETE( l_page_buffer );

    for (size_t ch = 0; ch <  DRS_CHANNELS_COUNT; ch ++){
        DAP_DELETE( l_sum_delta_ref[ch] );
        DAP_DELETE( l_stats[ch] );
    }

    DAP_DELETE( l_sum_delta_ref );
    DAP_DELETE( l_stats );
    DAP_DELETE( l_cells );
    return l_rc;
}

/**
 * double *buffer			массив с данными;
 * double *sumDeltaRef			массив дельт;
 * double *statistic			массив статистики по дельтам;
 * unsigned int shift			двиг в данных;
 */
static double s_get_deltas_min(double*a_buffer,double **a_sum_delta_ref,double **a_stats,unsigned int a_shift)
{
    unsigned int n, pz, pz1;
    double l_average[1] = {0};
    double l_vmin,l_vmax, l_vtmp,l_min;
    unsigned ch = 0;
    getAverageCh(l_average,a_buffer,DRS_CELLS_COUNT_CHANNEL,2,ch);
#define DRS_IDX(a,b) (b*DRS_CHANNELS_COUNT+a)

    if ((16383.0-l_average[ch]) > l_average[ch]) {
        l_vtmp = l_average[ch]/2.0;
    }else{
        l_vtmp=(16383.0-l_average[ch])/2.0;
    }
    l_vmin=l_average[ch]-l_vtmp;
    l_vmax=l_average[ch]+l_vtmp;

    for(n=0; n < DRS_CELLS_COUNT_CHANNEL ;n++) {
        pz=(a_shift+n)&1023;
        pz1=(n+1)&1023;
        if ((a_buffer[DRS_IDX(ch,n)]<=l_vmax) && (a_buffer[DRS_IDX(ch,pz1)]>=l_vmin) ) {
            a_sum_delta_ref[ch][pz]+=absf(a_buffer[DRS_IDX(ch,n)]-a_buffer[DRS_IDX(ch,pz1)]);
            a_stats[ch][pz]++;
        }
    }

    l_min=a_stats[0][0];
    for(n=0;n< DRS_CELLS_COUNT_CHANNEL;n++) {
        if(a_stats[ch][n] < l_min) {
            l_min = a_stats[ch][n];
        }
    }
    return l_min;
}
