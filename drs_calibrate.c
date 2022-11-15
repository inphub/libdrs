/*
 * drs_calibrate.c
 *
 *  Created on: 8 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <assert.h>
#include <pthread.h>

#include <dap_common.h>
#include <dap_string.h>

#include "drs.h"
#include "drs_calibrate.h"

#include "calibrate.h"
#include "commands.h"


#define LOG_TAG "drs_calibrate"


// Аргументы калибровки
struct args{
    union{
        struct{
            bool do_amplitude:1;
            bool do_time_local:1;
            bool do_time_global:1;
            unsigned padding:29;
        } DAP_ALIGN_PACKED;
        uint32_t raw; //  ключи калибровки, 1 бит амплитудная,2 локальная временная,3 глобальная временная
    } keys;
    drs_calibrate_params_t param;
    drs_calibrate_t * cal;
};

// Состояния калибровки (текущие )
drs_calibrate_t s_state[DRS_COUNT] = {};
// Поток калибровки
static void * s_thread_routine(void * a_arg);
static inline int s_run(int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params );

/**
 * @brief s_thread_routine
 * @param a_arg
 * @return
 */
static void * s_thread_routine(void * a_arg)
{
    assert(a_arg);
    struct args * l_args = (struct args*) a_arg;
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

    for(t=0;t<DCA_COUNT;t++){
        dap_string_append_printf(l_dca_shifts_str, "[%zd]=%f%s", t, l_args->param.ampl.levels[t+2],
                                 t<DCA_COUNT-1 ? "," : "");
    }
    dap_string_append(l_dca_shifts_str, "}");
    log_it(L_DEBUG, "DAC shifts: %s", l_dca_shifts_str->str);
    dap_string_free(l_dca_shifts_str, true);

    l_cal->progress = 10;

    setNumPages(1);
    l_cal->progress = 15;
    unsigned int l_value;
    //setSizeSamples(1024);//Peter fix
    if(l_args->keys.do_amplitude){
        log_it(L_DEBUG, "start amplitude calibrate. Levels %p, shifts %p\n",
               l_args->param.ampl.levels, l_args->param.ampl.levels+2 );
        l_value=calibrate_amplitude(&l_drs->coeffs,l_args->param.ampl.levels ,l_args->param.ampl.N ,g_ini, l_args->param.ampl.repeats,
                                     &l_cal->progress)&1;
        if(l_value==1){
            log_it(L_DEBUG, "end amplitude calibrate");
        }else{
            log_it(L_DEBUG, "amplitude calibrate error");
        }
    }
    if( l_args->keys.do_time_local ){
        log_it(L_DEBUG, "start time calibrate");
        l_value|=(calibrate_time( l_args->param.time_local.min_N,&l_drs->coeffs,g_ini)<<1)&2;
        if((l_value&2)==2){
            log_it(L_DEBUG, "end time calibrate");
        }else{
            log_it(L_DEBUG, "time calibrate error");
        }
    }

    l_cal->progress = 70;
    if( l_args->keys.do_time_global ){
        log_it(L_DEBUG, "start global time calibrate");
        l_value|=(calibrate_time_global(l_args->param.time_global.num_cycle ,&l_drs->coeffs,g_ini)<<2)&4;
        if((l_value&4)==4){
            log_it(L_DEBUG, "end global time calibrate");
        }else{
            log_it(L_DEBUG, "global time calibrate error");
        }
    }

    l_cal->progress = 100;

    pthread_rwlock_wrlock( &l_cal->rwlock);
    l_cal->is_running = false;
    pthread_rwlock_unlock( &l_cal->rwlock);

    DAP_DELETE(l_args);
    pthread_cond_broadcast(&l_cal->finished_cond);
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
    struct args * l_args  = DAP_NEW_Z(struct args);
    l_args->cal = l_cal;
    l_args->keys.raw = a_cal_flags;
    memcpy(&l_args->param, a_params, sizeof(*a_params) );
    l_cal->is_running = true;
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
    if (a_wait_msec < 0){
        pthread_cond_wait( &s_state[a_drs_num].finished_cond, &s_state[a_drs_num].finished_mutex );
    }else{
        struct timespec l_ts = {
            .tv_sec = a_wait_msec / 1000,
            .tv_nsec = (a_wait_msec % 1000) * 1000000
        };
        pthread_cond_timedwait( &s_state[a_drs_num].finished_cond, &s_state[a_drs_num].finished_mutex, &l_ts );
    }
    pthread_mutex_unlock(&s_state[a_drs_num].finished_mutex);
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

    int l_ret = -1;
    pthread_rwlock_rdlock(&l_cal->rwlock);
    if (l_cal->is_running){
        l_ret = (int) l_cal->progress;
    }
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

