/*
 * drs_cal_time_global.c
 *
 *  Created on: 22 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>
#include <math.h>

#include <dap_common.h>
#include <dap_time.h>
#include <dap_strfuncs.h>
#include <dap_config.h>

#include "data_operations.h"

#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"
#include "drs_cal.h"
#include "drs_cal_pvt.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_local.h"
#include "drs_cal_time_global.h"

#define LOG_TAG  "drs_cal_time_global"


#define GeneratorFrequency 50 //MHz
#define DRS_SINUS_FREQ  2.4*19
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];

//4.9152/2.4*19 || periodLength-длинна в отсчетах одного периода 4.9152 ГГц-частота ацп, 2400/19-МГц частота синуса
static const double s_period_length[] = {
    //[DRS_FREQ_5GHz] = 38.912
    [DRS_FREQ_1GHz] = (c_freq_DRS[DRS_FREQ_1GHz] * 1000.0 )/50.0,
    [DRS_FREQ_2GHz] = (c_freq_DRS[DRS_FREQ_2GHz] * 1000.0 )/50.0,
    [DRS_FREQ_3GHz] = (c_freq_DRS[DRS_FREQ_3GHz] * 1000.0 )/50.0,
    [DRS_FREQ_4GHz] = (c_freq_DRS[DRS_FREQ_4GHz] * 1000.0 )/50.0,
    [DRS_FREQ_5GHz] = (c_freq_DRS[DRS_FREQ_5GHz] * 1000.0 )/50.0
};

//26,315789473684210526315789473684- максимальное количество периодов в 1024 отсчетах->27, +1 для нуля;
static const unsigned s_period_max_count[] = {
  [DRS_FREQ_1GHz] = 1024/( ( c_freq_DRS[DRS_FREQ_1GHz] * 1000.0 )/50.0 ) + 2.0,
  [DRS_FREQ_2GHz] = 1024/( ( c_freq_DRS[DRS_FREQ_2GHz] * 1000.0 )/50.0 ) + 2.0,
  [DRS_FREQ_3GHz] = 1024/( ( c_freq_DRS[DRS_FREQ_3GHz] * 1000.0 )/50.0 ) + 2.0,
  [DRS_FREQ_4GHz] = 1024/( ( c_freq_DRS[DRS_FREQ_4GHz] * 1000.0 )/50.0 ) + 2.0,
    [DRS_FREQ_5GHz] = 1024/( ( c_freq_DRS[DRS_FREQ_5GHz] * 1000.0 )/50.0 ) + 2.0
    //[DRS_FREQ_5GHz] = 28
};

static bool s_debug_warns = false;
static bool s_debug_more = false;
static bool s_debug_dump_data = false;

static void s_collect_stats(drs_t * a_drs, atomic_uint_fast32_t * a_progress,unsigned a_iteration, double *a_x, double *a_y, double *a_sum_delta_ref, double *a_stats);

static int s_proc_drs(drs_t * a_drs, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);


/**
 * @brief drs_cal_time_init
 * @return
 */
int drs_cal_time_global_init()
{
    s_debug_more = dap_config_get_item_bool_default(g_config,"drs_cal_time_global","debug_more", s_debug_more);
    s_debug_warns = dap_config_get_item_bool_default(g_config,"drs_cal_time_global","debug_warns", s_debug_warns);
    s_debug_dump_data = dap_config_get_item_bool_default(g_config,"drs_cal_time_global","debug_dump_data", s_debug_dump_data);
    return 0;
}

/**
 * @brief drs_cal_time_global_deinit
 */
void drs_cal_time_global_deinit()
{

}

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
 * @brief s_y_idx
 * @param n
 * @param b
 */
static inline unsigned s_x_idx(unsigned n, unsigned b){
  return n + b*DRS_CELLS_COUNT_BANK;
};

/**
 * @brief drs_cal_time_global_apply  Применяет амплитудную калибровку к данным
 * @param a_drs                      Объект DRS
 * @param a_x                        Массив входящих значений
 * @param a_result                  Массив с результатами
 */
void drs_cal_time_global_apply( drs_t * a_drs,  double *a_in,   double *a_out )
{
    double l_current_x = 0.0;
    a_out[0]=0.0;
    for(unsigned b=0;b<DRS_CHANNEL_BANK_COUNT ;b++){
        for(unsigned n=0, pz_bank, pz;n< DRS_CELLS_COUNT_BANK;n++){
            //pz = ( a_drs->shift + n)&1023;
            pz = n + b*DRS_CELLS_COUNT_BANK;
            pz_bank = pz & 1023;

            if (n == 0 && b == 0)
                continue;
            else if(n == 0 && b != 0)
                l_current_x += ((a_in[s_x_idx(n,b)] - a_in[s_x_idx(DRS_CELLS_COUNT_BANK-1,b-1)])* a_drs->coeffs.kTime[ pz_bank ]);
            else
                l_current_x += ((a_in[s_x_idx(n,b)] - a_in[s_x_idx(n-1,b)])* a_drs->coeffs.kTime[ pz_bank ]);
            a_out[ s_x_idx(n,b) ] = l_current_x;
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

    // Выставляем прогресс
    unsigned l_progress_old = 0;
    double l_progress_total = a_args->cal->progress_per_stage;
    double l_progress_step = l_progress_total / ((double) a_args->param.time_global.num_cycle);
    if (a_progress)
      l_progress_old = *a_progress;

    // Проверяем на предмет того, что была сделана предыдущая калибровка
    if(( a_drs->coeffs.indicator&3)!=3){// TODO заменить на что-то более вменяемое
        log_it(L_WARNING,"before global timer calibration you need to do the timer calibration and amplitude calibration");
        return -1;
    }
    dap_nanotime_t l_ts_start = dap_nanotime_now();



    // Включаем режим таймерной калибровки и генератор синуса
    drs_mode_t l_mode_old = drs_get_mode(a_drs->id);
    drs_set_mode(a_drs->id, DRS_MODE_CAL_TIME);
    drs_set_sinus_signal(true);


    for( unsigned i = 0; i < a_args->param.time_global.num_cycle; i++){
        int l_ret = drs_data_get_page_from_first(a_drs,DRS_OP_FLAG_SOFT_START, l_y_raw,sizeof (l_y_raw) );
        if ( l_ret != 0){
            log_it(L_ERROR,"data get returned with error, code %d", l_ret);
            drs_set_sinus_signal(false);
            drs_set_mode(a_drs->id, l_mode_old);
            break;
        }
        if ( s_debug_dump_data ){
            static bool l_do_once = true;
            if (l_do_once){
                char * l_file_name = dap_strdup_printf("time_global_good_dump_first_y_raw_shift_%u", drs_get_shift(a_drs->id,0));
                drs_data_dump_in_files_unsigned(l_file_name, (unsigned *) l_y_raw, DRS_CELLS_COUNT/DRS_CHANNELS_COUNT,
                                      DRS_DATA_DUMP_CSV |  DRS_DATA_DUMP_BIN |
                                      DRS_DATA_DUMP_ADD_TIMESTAMP | DRS_DATA_DUMP_ADD_PATH_VAR_LIB );
                DAP_DELETE(l_file_name);


                l_do_once = false;
            }
        }

        drs_cal_y_apply(a_drs, l_y_raw,l_y, DRS_CAL_APPLY_Y_CELLS | DRS_CAL_APPLY_Y_SPLASHS );


        // Заполняем массив X
        for (unsigned n = 0; n < DRS_CELLS_COUNT_CHANNEL; n++){
            l_x[n] = n;
        }
        drs_cal_time_local_apply(a_drs, l_x, l_x);

        s_collect_stats(a_drs,  a_progress, i, l_x,l_y, l_sum_delta_ref,l_stats);
        if (a_progress)
            *a_progress = l_progress_old + ((unsigned) floor(((double) i)*l_progress_step));

    }


    // Выключаем режим синуса и возвращаемся в обычный режим
    drs_set_sinus_signal(false);
    drs_set_mode(a_drs->id, l_mode_old);

    // Считаем коэфициенты
    for(unsigned n = 0; n < DRS_CELLS_COUNT_BANK; n++) {
        if(l_stats[n] == 0.0){
            l_stats[n] = 1.0;
        }
        if ( n < 10 ){
            debug_if(s_debug_more,L_INFO, "SumDeltaRef=%f, stats=%f", l_sum_delta_ref[n], l_stats[n]);
        }
        a_drs->coeffs.kTime[n] = l_sum_delta_ref[n] / l_stats[n];
    }

    // Вычисляем время работы калибровки
    double l_ts_diff  =  ((double) (dap_nanotime_now() - l_ts_start))/ 1000000000.0  ;
    log_it(L_NOTICE,"Finished local time calibration in %.3f seconds", l_ts_diff);

     a_drs->coeffs.indicator |= 4;
    // Обновляем прогресс бар
    if (a_progress)
      *a_progress = l_progress_old +((unsigned)floor(l_progress_total));


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
static void s_collect_stats(drs_t * a_drs, atomic_uint_fast32_t * a_progress,unsigned a_iteration, double *a_x, double *a_y, double *a_sum_delta_ref, double *a_stats)
{

    double average, l_last_x = 0.0 ,l_last_y = 0.0, deltX = 0.0;

    unsigned int pz,l_count=0;

    // Запоминаем максимальное число периодов и длину отдельного периода для данной выбранной частоты
    const unsigned l_max_period_count = s_period_max_count [g_current_freq];
    const double l_period_length = s_period_length [g_current_freq];

    // TODO заменить с выделения кучи на выделение стека и сравнить скорость
    double * l_zero_cross_x = DAP_NEW_Z_SIZE(double, sizeof(double) * l_max_period_count );
    double * l_period_delt = DAP_NEW_Z_SIZE(double, sizeof(double) * l_max_period_count );
    unsigned *l_indexs = DAP_NEW_Z_SIZE(unsigned, sizeof(unsigned) * l_max_period_count );

    // Считаем среднее по y
    average = drs_ch_get_average(a_y,DRS_CELLS_COUNT_BANK,DRS_CHANNEL_9 );
    //average = 8192;
    // Бежим по ячейкам

    size_t l_cells_count = DRS_CELLS_COUNT_BANK - g_drs_data_cut_from_begin ;

    for(unsigned n=0;n< l_cells_count && l_count < l_max_period_count; n++){
        unsigned n_9idx = n* DRS_CHANNELS_COUNT + DRS_CHANNEL_9;
        if( (average >= l_last_y) &&
            (a_y[n_9idx] >= average) &&
            (n != 0) ) {

            deltX = a_x[n] - l_last_x;
            if(a_x[n] < l_last_x) {
                deltX += 1024.0;
            }

            l_zero_cross_x[l_count] =  deltX / (a_y[n_9idx]-l_last_y) * (average - l_last_y)  + l_last_x;
            if(l_count > 0) {
                l_period_delt[l_count-1] = l_zero_cross_x[l_count] - l_zero_cross_x[l_count - 1];
                if (s_debug_dump_data){
                    static bool l_wasnt_good_dumped = true;
                    if (l_count>2 && l_period_delt[l_count-1] >= 90 && l_wasnt_good_dumped){
                        char * l_file_name = dap_strdup_printf("time_global_good_dump_y_shift_%u", drs_get_shift(a_drs->id,0));
                        drs_data_dump_in_files_double(l_file_name, a_y, DRS_CELLS_COUNT,
                                              DRS_DATA_DUMP_CSV |  DRS_DATA_DUMP_BIN |
                                              DRS_DATA_DUMP_ADD_TIMESTAMP | DRS_DATA_DUMP_ADD_PATH_VAR_LIB );
                        DAP_DELETE(l_file_name);
                        l_wasnt_good_dumped = false;
                    }
                    s_debug_dump_data = false;
                    s_debug_warns = false;
                }
                if ( l_period_delt[l_count-1] < 90){
                    if (s_debug_warns)
                        log_it(L_WARNING,"l_period_delt[%u] = %f",l_count-1,l_period_delt[l_count-1] );

                    if (s_debug_dump_data){
                        char * l_file_name = dap_strdup_printf("time_global_bad_dump_y_shift_%u", drs_get_shift(a_drs->id,0));
                        drs_data_dump_in_files_double(l_file_name, a_y, DRS_CELLS_COUNT,
                                              DRS_DATA_DUMP_CSV |  DRS_DATA_DUMP_BIN |
                                              DRS_DATA_DUMP_ADD_TIMESTAMP | DRS_DATA_DUMP_ADD_PATH_VAR_LIB );
                        DAP_DELETE(l_file_name);
                    }
                }
            }
            l_indexs[l_count] = a_drs->shift_bank + n;
            if(l_indexs[l_count] > DRS_CELLS_COUNT_CHANNEL){
                log_it(L_WARNING,"Wrong index %u, can't be more than %u", l_indexs[l_count],DRS_CELLS_COUNT_CHANNEL );
            }
            l_count++;
        }

        l_last_y = a_y[n_9idx];
        l_last_x = a_x[n];

    }

    // Бежим по подсчитанному количеству
    for(unsigned n = 1; n < l_count; n++) {
        if( l_indexs[n] > l_indexs[n-1] ){
            for(unsigned l = l_indexs[n-1]; l < l_indexs[n]; l++){
                if(l_indexs[n] > DRS_CELLS_COUNT_CHANNEL)
                    log_it(L_WARNING,"Wrong index %u, can't be more than %u", l_indexs[n],DRS_CELLS_COUNT_CHANNEL );
                pz=l&1023;

                a_sum_delta_ref[pz] += l_period_length / l_period_delt[n-1];
                a_stats[pz]++;
                //if( a_stats[pz] == a_iteration )
                //    log_it(L_INFO,"Достигли потолка на позиции %u", pz);
            }
        }else if( l_indexs[n] == l_indexs[n-1] ){
            pz=l_indexs[n]&1023;
            a_sum_delta_ref[pz] += l_period_length / l_period_delt[n-1];
            a_stats[pz]++;
        }
    }
    if(s_debug_more){
        log_it(L_NOTICE,"-- stats dump -- ");
        for(unsigned n = 1; n < l_count ; n++) {
            log_it(L_DEBUG, "%u: zero_cross_x=%f period_delt=%f",n-1,l_zero_cross_x[n-1], l_period_delt[n-1]);
        }
    }

    // Очищаем память
    DAP_DELETE(l_zero_cross_x);
    DAP_DELETE(l_period_delt);
    DAP_DELETE(l_indexs);
}


