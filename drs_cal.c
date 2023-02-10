/*
 * drs_cal.c
 *
 *  Created on: 8 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>
#include <pthread.h>

#include <dap_common.h>
#include <dap_string.h>

#include "drs.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_global.h"
#include "drs_cal_time_local.h"

#include "calibrate.h"
#include "commands.h"
#include "data_operations.h"

#define LOG_TAG "drs_cal"


#define GeneratorFrequency 50 //MHz
#define periodLength 38.912 //4.9152/2.4*19 || periodLength-длинна в отсчетах одного периода 4.9152 ГГц-частота ацп, 2400/19-МГц частота синуса
#define maxPeriodsCount 28//26,315789473684210526315789473684- максимальное количество периодов в 1024 отсчетах->27, +1 для нуля;
//#define freqDRS 4915200.0/1000000000.0
//extern const double freqDRS[];

/**
 * double*x -массив под знчаения X
 * unsigned int shift - сдвиг ячеек
 * coefficients *coef - струтурка с коэффициентами
 * unsigned int key - ключ применения калибровок 4 бит-локальная временная, 5 бит-глобальная временная, 6 бит-приведение к физическим величинам
 */

// Состояния калибровки (текущие )
drs_calibrate_t s_state[DRS_COUNT] = {};
// Поток калибровки
static void *     s_thread_routine(void * a_arg);
static inline int s_run           (int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params );

static void       s_x_to_real     (double* x                                                             );
static void       s_remove_splash (drs_t * a_drs, double* a_Y, bool a_ch9_only                           );

/**
 * @brief s_thread_routine
 * @param a_arg
 * @return
 */
static void * s_thread_routine(void * a_arg)
{
    assert(a_arg);
    drs_cal_args_t * l_args = (drs_cal_args_t*) a_arg;
    drs_calibrate_t * l_cal = l_args->cal;
    assert(l_cal);
    drs_t * l_drs = l_cal->drs;
    assert(l_drs);

    l_cal->progress = 5;

    log_it( L_NOTICE, "Start calibration for DRS #%u", l_drs->id);
    log_it(L_DEBUG, "Amplitude  : repeats_count=%u levels_count=%u begin=%f end=%f",
           l_args->param.ampl.repeats, l_args->param.ampl.N, l_args->param.ampl.levels[0], l_args->param.ampl.levels[1]);
    log_it(L_DEBUG, "Time local : min_N=%u", l_args->param.time_local.min_N);
    log_it(L_DEBUG, "Time global: num_cycle=%u", l_args->param.time_global.num_cycle);


    dap_string_t * l_dca_shifts_str = dap_string_new("{");
    size_t t;

    for(t=0;t<DRS_CHANNELS_COUNT ;t++){
        dap_string_append_printf(l_dca_shifts_str, "[%zd]=%f%s", t, l_args->param.ampl.levels[t],
                                 t<DRS_CHANNELS_COUNT-1 ? "," : "");
    }
    dap_string_append(l_dca_shifts_str, "}");
    log_it(L_DEBUG, "DAC shifts: %s", l_dca_shifts_str->str);
    dap_string_free(l_dca_shifts_str, true);

    l_cal->progress = 10;

    //drs_set_num_pages(l_cal->drs, 1);
    l_cal->progress = 15;
    //setSizeSamples(1024);//Peter fix
    if(l_args->keys.do_amplitude){
        log_it(L_NOTICE, "start amplitude calibrate. Levels %p, shifts %p\n",
               l_args->param.ampl.levels, l_args->param.ampl.levels+2 );
        int l_ret = drs_cal_amp( l_cal->drs->id, l_args, &l_cal->progress);
        if(l_ret == 0){
            log_it(L_INFO, "end amplitude calibrate");
        }else{
            log_it(L_ERROR, "amplitude calibrate error, code %d", l_ret);
        }
    }
    if( l_args->keys.do_time_local ){
        log_it(L_NOTICE, "start time local calibrate");
        int l_ret = drs_cal_time_local(l_cal->drs->id, l_args, &l_cal->progress);
        if(l_ret == 0){
            log_it(L_INFO, "end time local calibrate");
        }else{
            log_it(L_ERROR, "time local calibrate error, code %d", l_ret);
        }
    }

    l_cal->progress = 70;
    if( l_args->keys.do_time_global ){
        log_it(L_NOTICE, "start time global calibrate");
        int l_ret = drs_cal_time_global(l_cal->drs->id, l_args, &l_cal->progress);
        if(l_ret == 0){
            log_it(L_INFO, "end time global calibrate");
        }else{
            log_it(L_ERROR, "time global calibrate error, code %d", l_ret);
        }
    }

    l_cal->progress = 100;

    pthread_rwlock_wrlock( &l_cal->rwlock);
    l_cal->is_running = false;
    l_cal->ts_end = dap_nanotime_now();
    pthread_rwlock_unlock( &l_cal->rwlock);

    DAP_DELETE(l_args);
    pthread_mutex_lock(&l_cal->finished_mutex);
    pthread_cond_broadcast(&l_cal->finished_cond);
    pthread_mutex_unlock(&l_cal->finished_mutex);

    return NULL;
}


/**
 * @brief  Инициализирует модуль
 * @return
 */
int drs_calibrate_init()
{
    for(size_t i = 0; i< DRS_COUNT; i++){
        pthread_rwlock_init(& s_state[i].rwlock, NULL);
        s_state[i].drs = &g_drs[i];
    }
    return 0;
}

/**
 * @brief Денициализирует модуль
 */
void drs_calibrate_deinit()
{
    for(size_t i = 0; i< DRS_COUNT; i++)
        pthread_rwlock_destroy(& s_state[i].rwlock);
}

/**
 * @brief s_run
 * @param a_drs_num
 * @param a_cal_flags
 * @param a_params
 * @return
 */
static inline int s_run(int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params )
{
    drs_calibrate_t * l_cal = &s_state[a_drs_num];

    // Проверяем, запущена ли уже калибровка
    pthread_rwlock_rdlock(&l_cal->rwlock);
    if ( l_cal->is_running){
        log_it(L_WARNING, "DRS #%d is already running calibration (%u%% done)", a_drs_num, l_cal->progress );
        pthread_rwlock_unlock(&l_cal->rwlock);
        return -2;
    }
    pthread_rwlock_unlock(&l_cal->rwlock);

    // Запускаем поток с калибровкой
    pthread_rwlock_wrlock(&l_cal->rwlock);

    struct drs_cal_args * l_args  = DAP_NEW_Z(struct drs_cal_args);
    l_args->cal = l_cal;
    l_args->keys.raw = a_cal_flags;
    memcpy(&l_args->param, a_params, sizeof(*a_params) );

    l_cal->is_running = true;
    l_cal->ts_start = dap_nanotime_now();
    pthread_create(&l_cal->thread_id, NULL, s_thread_routine, l_args);
    pthread_rwlock_unlock(&l_cal->rwlock);

    return 0;
}


/**
 * @brief drs_calibrate_run
 * @param a_drs_num
 */
int drs_calibrate_run(int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params )
{
    if (a_drs_num == -1 ){
        int l_ret = 0;
        for (size_t n = 0; n < DRS_COUNT; n ++){
            l_ret = s_run(a_drs_num, a_cal_flags, a_params);
            if(l_ret != 0)
                break;
        }
        return l_ret;
    }else if (a_drs_num >=0 ){
        if(a_drs_num >= DRS_COUNT){
            log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
            return -1;
        }
        return s_run(a_drs_num, a_cal_flags, a_params);
    }else{
        log_it(L_ERROR, "Wrong DRS number %d",a_drs_num);
        return -2;
    }
}

/**
 * @brief drs_calibrate_wait_for_finished
 * @param a_drs_num
 * @param a_wait_msec
 * @return
 */
int drs_calibrate_wait_for_finished(int a_drs_num, int a_wait_msec)
{
    if(a_drs_num >= DRS_COUNT){
        log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
        return -1;
    }
    drs_calibrate_t * l_cal = &s_state[a_drs_num];

    // Блокируем вызов сигнала о том, что запущены
    pthread_mutex_lock(& l_cal->finished_mutex);

    // Проверяем, не запущен ли
    pthread_rwlock_rdlock(&l_cal->rwlock);
    bool l_is_running = l_cal->is_running;
    pthread_rwlock_rdlock(&l_cal->rwlock);
    if( ! l_is_running){
        pthread_mutex_unlock(& l_cal->finished_mutex);
        return 0;
    }

    // Встаём в ожидание срабатывания события
    if (a_wait_msec < 0){
        pthread_cond_wait( &l_cal->finished_cond, &l_cal->finished_mutex );
    }else{
        struct timespec l_ts = {
            .tv_sec = a_wait_msec / 1000,
            .tv_nsec = (a_wait_msec % 1000) * 1000000
        };
        pthread_cond_timedwait( &l_cal->finished_cond, &l_cal->finished_mutex, &l_ts );
    }
    pthread_mutex_unlock(&l_cal->finished_mutex);
    return 0;
}


/**
 * @brief drs_calibrate_get_state
 * @param a_drs_num
 * @return
 */
drs_calibrate_t* drs_calibrate_get_state(int a_drs_num)
{
    if(a_drs_num >= DRS_COUNT){
        log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
        return NULL;
    }
    return &s_state[a_drs_num];
}

/**
 * @brief drs_calibrate_progress
 * @param a_drs_num
 * @return
 */
int drs_calibrate_progress(int a_drs_num)
{
    drs_calibrate_t * l_cal = drs_calibrate_get_state(a_drs_num);
    if (l_cal == NULL)
        return -2;
    pthread_rwlock_rdlock(&l_cal->rwlock);
    int l_ret = (int) l_cal->progress;
    pthread_rwlock_unlock(&l_cal->rwlock);
    return l_ret;
}

/**
 * @brief drs_calibrate_is_running
 * @param a_drs_num
 * @return
 */
bool drs_calibrate_is_running(int a_drs_num)
{
    drs_calibrate_t * l_cal = drs_calibrate_get_state(a_drs_num);
    if (l_cal == NULL)
        return -2;

    bool l_ret;
    pthread_rwlock_rdlock(&l_cal->rwlock);
    l_ret = l_cal->is_running;
    pthread_rwlock_unlock(&l_cal->rwlock);
    return l_ret;

}

/**
 * @brief drs_calibrate_abort
 * @param a_drs_num
 */
int drs_calibrate_abort(int a_drs_num)
{
    // Проверка на номер DRS
    if(a_drs_num >= DRS_COUNT){
        log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
        return -2;
    }

    pthread_rwlock_rdlock(&s_state[a_drs_num].rwlock);

    // Калибровка вообще идёт?
    if( !s_state[a_drs_num].is_running){
        log_it(L_WARNING, "DRS #%d is not running, nothing to abort", a_drs_num);
        return -1;
    }

    // Грохаем поток
    pthread_cancel( s_state[a_drs_num].thread_id);

    pthread_rwlock_unlock(&s_state[a_drs_num].rwlock);

    // Обнуляем состояния
    pthread_rwlock_rdlock(&s_state[a_drs_num].rwlock);
    s_state[a_drs_num].is_running = false;
    s_state[a_drs_num].thread_id  = 0;
    s_state[a_drs_num].progress   = 0;
    pthread_rwlock_unlock(&s_state[a_drs_num].rwlock);
    return 0;
}


/**
 * @brief s_x_to_real
 * @param x
 */
static void s_x_to_real(double*x)
{
    *x=(*x)/(drs_get_freq_value(g_current_freq));
}


/**
 * @brief drs_cal_x_amply
 * @param a_drs
 * @param a_x
 * @param a_flags
 */
void drs_cal_x_apply(drs_t * a_drs, double*a_x, int a_flags)
{
    if(a_flags & DRS_CAL_APPLY_X_TIME_LOCAL) {
        drs_cal_time_local_apply(a_drs, a_x, a_x);
    }
    /*if(a_flags & DRS_CAL_APPLY_X_TIME_GLOBAL) {
        drs_cal_time_global_apply(a_drs, a_x,a_x);
    }
    if(a_flags & DRS_CAL_APPLY_PHYS) {
        do_on_array(a_x,DRS_CELLS_COUNT_ALL,s_x_to_real);
    }*/

}


/*
 * Применяет амплитудную калибровку к данным
 * unsigned short *buffer			массив данных;
 * double *dBuf 					массив данных с результатом применения амплитудной калибровки
 * unsigned int shift 				сдвиг получаемый через getShiftIndex;
 * coefficients *coef				структура с кэффициентами;
 * unsigned int chanalLength		длинна массива для 1 канала;
 * unsigned int chanalCount			количество каналов
 * unsigned int a_flags	    		0 бит- применение калибровки для ячеек, 1 бит- межканальная калибровка,2 бит- избавление от всплесков, 3 бит- приведение к физическим виличинам
 * */
void drs_cal_y_apply(drs_t * a_drs, unsigned short *a_in,double *a_out, int a_flags)
{
    unsigned int l_ch_id,l_cell_id,koefIndex;

    // Если мы сейчас в режиме 9ого канала, то автоматически взводим этот флаг
    if (drs_get_mode(a_drs->id) == DRS_MODE_CAL_TIME)
        a_flags |= DRS_CAL_APPLY_CH9_ONLY;

    unsigned l_cells_proc_count = a_flags & DRS_CAL_APPLY_CH9_ONLY ? DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL;
    //double * l_bi = s_bi; // a_drs->coeffs.b
    //double * l_ki = s_ki; // a_drs->coeffs.k
    //double  **l_bi = a_drs->coeffs.b;
    //double  **l_ki = a_drs->coeffs.k;
    //double average[4];
    //getAverageInt(average,buffer,DRS_CELLS_COUNT_CHANNEL,DRS_CHANNELS_COUNT);
    for(l_ch_id=0; l_ch_id<DRS_CHANNELS_COUNT;l_ch_id++){
        if(a_flags & DRS_CAL_APPLY_CH9_ONLY)
            l_ch_id = DRS_CHANNEL_9;

        for(l_cell_id=0; l_cell_id<l_cells_proc_count;l_cell_id++){
            size_t l_cell_id_masked = a_flags & DRS_CAL_APPLY_CH9_ONLY ? l_cell_id :
                                                                        l_cell_id&3072 ;
            unsigned l_inout_id = l_cell_id * DRS_CHANNELS_COUNT + l_ch_id;

            koefIndex=  a_flags & DRS_CAL_APPLY_CH9_ONLY ?
                (a_drs->shift + l_cell_id ) & 1023
                :(a_drs->shift + (l_cell_id&1023) ) & 1023 ;
            a_out[l_inout_id] = a_in[l_inout_id];
            if((a_flags & DRS_CAL_APPLY_Y_CELLS)!=0){
                double l_bi, l_ki;
                if (a_flags & DRS_CAL_APPLY_CH9_ONLY){
                    l_bi = a_drs->coeffs.b9 [koefIndex ];
                    l_ki = a_drs->coeffs.k9 [koefIndex ];
                }else{
                    l_bi = a_drs->coeffs.b[l_ch_id][koefIndex | l_cell_id_masked];
                    l_ki = a_drs->coeffs.k[l_ch_id][koefIndex | l_cell_id_masked];
                }


                a_out[l_inout_id] =  ( a_out[l_inout_id] - l_bi ) /
                                                 (l_ki +1.0 );
            }
            if((a_flags & DRS_CAL_APPLY_Y_INTERCHANNEL)!=0){
                a_out[l_inout_id] = (a_out[l_inout_id] - a_drs->coeffs.chanB[l_ch_id] ) / a_drs->coeffs.chanK[l_ch_id];
            }
            if((a_flags & DRS_CAL_APPLY_PHYS)!=0){
                a_out[l_inout_id]=(a_out[l_inout_id]-g_ini->fastadc.adc_offsets[l_ch_id])/g_ini->fastadc.adc_gains[l_ch_id];
            }
        }
        if(a_flags & DRS_CAL_APPLY_CH9_ONLY)
            break;
    }
    if((a_flags& DRS_CAL_APPLY_Y_SPLASHS)!=0)
    {
        s_remove_splash(a_drs, a_out, a_flags & DRS_CAL_APPLY_CH9_ONLY);
    }
}

/**
 * @brief drs_cal_state_print
 * @param a_reply
 * @param a_cal
 * @param a_limits
 * @param a_flags
 */
void drs_cal_state_print(dap_string_t * a_reply, drs_calibrate_t *a_cal, unsigned a_limits, int a_flags)
{
    pthread_rwlock_rdlock(&a_cal->rwlock);
    dap_string_append_printf( a_reply, "Running:     %s\n\n", a_cal->is_running? "yes" : "no" );
    dap_string_append_printf( a_reply, "Progress:    %d%%\n\n", a_cal->progress );
    if (a_cal->ts_end){
        coefficients_t * l_params = &a_cal->drs->coeffs;
        if ( a_flags & DRS_COEF_SPLASH)
            dap_string_append_array(a_reply, "splash", "0x%08X", l_params->splash, a_limits);


        if ( a_flags & DRS_COEF_DELTA_TIME )
            dap_string_append_array(a_reply, "deltaTimeRef", "%f", l_params->deltaTimeRef, a_limits);

        if ( a_flags & DRS_COEF_CHAN_K )
            dap_string_append_array(a_reply, "chanK", "%f", l_params->chanK, a_limits);

        if ( a_flags & DRS_COEF_CHAN_B )
            dap_string_append_array(a_reply, "chanB", "%f", l_params->chanB, a_limits);

        if ( a_flags & DRS_COEF_K_TIME )
            dap_string_append_array(a_reply, "kTime", "%f", l_params->kTime, a_limits);

        if ( a_flags & DRS_COEF_K ){
            char l_str[128];
            for(unsigned c = 0; c< DRS_CHANNELS_COUNT; c++){
                snprintf(l_str, sizeof(l_str),"k[%u]", c);
                dap_string_append_array(a_reply, l_str, "%f", l_params->k[c], a_limits);
            }
        }

        if ( a_flags & DRS_COEF_B ){
            char l_str[128];
            for(unsigned c = 0; c< DRS_CHANNELS_COUNT; c++){
                snprintf(l_str, sizeof(l_str),"b[%u]", c);
                dap_string_append_array(a_reply, l_str, "%f", l_params->b[c], a_limits);
            }
        }

        if ( a_flags & DRS_COEF_K9 )
            dap_string_append_array(a_reply, "k9", "%f", l_params->k9, a_limits);

        if ( a_flags & DRS_COEF_B9 )
            dap_string_append_array(a_reply, "b9", "%f", l_params->b9, a_limits);

        dap_string_append_printf( a_reply, "indicator=%d \n\n", l_params->indicator);
        dap_string_append_printf( a_reply, "Got time:    %.3f seconds \n\n",
                                   ((double) (a_cal->ts_end - a_cal->ts_start)) / 1000000000.0 );


    }
    pthread_rwlock_unlock(&a_cal->rwlock);
}


/**
 * @brief s_remove_splash
 * @param Y
 * @param shift
 * @param coef
 */
static void s_remove_splash(drs_t * a_drs, double* a_Y, bool a_ch9_only)
{
    size_t i=0,j=0;
    for(j=0;j<DRS_CHANNELS_COUNT;j++){
        #define A_Y_IDX(a) ( (a)*DRS_CHANNELS_COUNT+j)
        i = ( a_drs->coeffs.splash[j] - a_drs->shift + 1023) & 1023;
        //if((i > 0) && (i < 1023)){
        //    a_Y[A_Y_IDX(i)] = (a_Y[A_Y_IDX(i+1)] + a_Y[a_Y[A_Y_IDX(i-1)]) / 2;
        //}

        if(i == 0){
            a_Y[j] = (a_Y [4*1 + j] + a_Y[4*2 + j]) / 2;
        }

        if(i == 1023){
            a_Y[1023 * 4 + j]=(a_Y[j] + a_Y[1022 * 4 + j]) / 2;
        }
    }
}
