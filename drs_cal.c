/*
 * drs_cal.c
 *
 *  Created on: 8 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#include <errno.h>
#include <assert.h>
#include <pthread.h>

#include <dap_sdk.h>
#include <dap_common.h>
#include <dap_config.h>
#include <dap_string.h>
#include <dap_time.h>
#include <dap_file_utils.h>

#include "drs.h"
#include "drs_data.h"
#include "drs_cal.h"
#include "drs_cal_pvt.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_global.h"
#include "drs_cal_time_local.h"

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

struct drs_cal_apply_flags g_drs_cal_apply_to_str[]={
  [0]        = {"CELLS",       "применение калибровки для ячеек"},
  [1]        = {"INTERCHANNEL","межканальная калибровка"},
  [2]        = {"SPLASHS","избавление от всплесков"},
  [8]        = {"TIME_LOCAL", "Временная локальная калибровка"},
  [9]        = {"TIME_GLOBAL", "Временная глобальная калибровка"},
  [29]       = {"EQUALIZE","Выровнять итоговые результаты"},
  [30]       = {"ROTATE","Развернуть итоговые результаты"},
  [28]       = {"PHYS","приведение к физическим величинам"},
  [31]       = {"CH9_ONLY","Только 9ый канал"},
};

// Состояния калибровки (текущие )
drs_calibrate_t s_state[DRS_COUNT] = {};

static char * s_cal_file_path = NULL;
static bool s_debug_more = false;
static unsigned s_splash_treshold = DRS_CAL_SPLASH_TRESHOLD_DEFAULT;

// Поток калибровки
static void *     s_thread_routine(void * a_arg);
static inline int s_run           (int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params );

static void       s_x_to_real     (double* x                                                             );
static void       s_remove_splash (drs_t * a_drs, double* a_Y, bool a_ch9_only                           );

static void s_cal_file_path_update();

/**
 * @brief drs_cal_set_splash_treshold
 * @param a_treshold
 */
void drs_cal_set_splash_treshold(unsigned a_treshold)
{
    s_splash_treshold = a_treshold;
}

/**
 * @brief drs_cal_get_splash_treshold
 */
unsigned drs_cal_get_splash_treshold()
{
  return s_splash_treshold;
}

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


    unsigned l_stages = 0;
    if (l_args->keys.do_amplitude)
      l_stages++;

    if (l_args->keys.do_time_global)
      l_stages++;

    if (l_args->keys.do_time_local)
      l_stages++;

    l_cal->progress = 0.0;
    l_cal->progress_per_stage = l_stages?  100.0 / (double) l_stages : 100.0;

    log_it( L_NOTICE, "Start calibration for DRS #%u (progress per stage %f%%", l_drs->id, l_cal->progress_per_stage);
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

    //setSizeSamples(1024);//Peter fix
    if(l_args->keys.do_amplitude){
        log_it(L_NOTICE, "start amplitude calibrate. Levels %p, shifts %p\n",
               l_args->param.ampl.levels, l_args->param.ampl.levels+2 );
        unsigned l_progress_old = l_cal->progress;
        l_cal->stage = DRS_CAL_FLAG_AMPL_CELL;
        int l_ret = drs_cal_amp( l_cal->drs->id, l_args, &l_cal->progress);

        if(l_ret == 0){
            log_it(L_INFO, "end amplitude calibrate");
        }else{
            log_it(L_ERROR, "amplitude calibrate error, code %d", l_ret);
        }
        l_cal->progress = l_progress_old + ((unsigned) floor(l_args->cal->progress_per_stage));
    }

    if( l_args->keys.do_time_local ){
        log_it(L_NOTICE, "start time local calibrate");
        unsigned l_progress_old = l_cal->progress;
        l_cal->stage = DRS_CAL_FLAG_TIME_LOCAL;
        int l_ret = drs_cal_time_local(l_cal->drs->id, l_args, &l_cal->progress);
        if(l_ret == 0){
            log_it(L_INFO, "end time local calibrate");
        }else{
            log_it(L_ERROR, "time local calibrate error, code %d", l_ret);
        }
        l_cal->progress = l_progress_old + ((unsigned) floor(l_args->cal->progress_per_stage));
    }

    if( l_args->keys.do_time_global ){
        log_it(L_NOTICE, "start time global calibrate");
        unsigned l_progress_old = l_cal->progress;
        l_cal->stage = DRS_CAL_FLAG_TIME_GLOBAL;
        int l_ret = drs_cal_time_global(l_cal->drs->id, l_args, &l_cal->progress);
        if(l_ret == 0){
            log_it(L_INFO, "end time global calibrate");
        }else{
            log_it(L_ERROR, "time global calibrate error, code %d", l_ret);
        }
        l_cal->progress = l_progress_old + ((unsigned) floor(l_args->cal->progress_per_stage));
    }

    l_cal->progress = 100;

    pthread_rwlock_wrlock( &l_cal->rwlock);
    l_cal->is_running = false;
    l_cal->ts_end = dap_nanotime_now();
    l_cal->stage = 0;
    pthread_rwlock_unlock( &l_cal->rwlock);

    DAP_DELETE(l_args);
    pthread_mutex_lock(&l_cal->finished_mutex);
    pthread_cond_broadcast(&l_cal->finished_cond);
    pthread_mutex_unlock(&l_cal->finished_mutex);

    drs_cal_save(s_cal_file_path);

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


    s_debug_more = dap_config_get_item_bool_default(g_config,"debug","drs_cal", false);

    drs_cal_file_path_update();
    drs_cal_load();

    drs_cal_amp_init();
    drs_cal_time_local_init();
    drs_cal_time_global_init();

    return 0;
}

/**
 * @brief drs_cal_file_path_update
 */
void drs_cal_file_path_update()
{
  dap_string_t * l_file_coeffs = dap_string_new(g_dap_vars.core.sys_dir);
  dap_string_append(l_file_coeffs,"/var/lib/");
  dap_mkdir_with_parents(l_file_coeffs->str);
  dap_string_append_printf(l_file_coeffs,"/cal_coeffs_freq_%sGHz.bin",c_freq_str_short[g_current_freq] );

  DAP_DEL_Z(s_cal_file_path);
  s_cal_file_path = dap_string_free(l_file_coeffs, false);
}

/**
 * @brief Денициализирует модуль
 */
void drs_calibrate_deinit()
{
  drs_cal_time_global_deinit();
  drs_cal_time_local_deinit();
  drs_cal_amp_deinit();

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
 * @brief drs_cal_get_stage
 * @param a_drs_num
 */
unsigned drs_cal_get_stage(int a_drs_num)
{
    assert(a_drs_num <0 || a_drs_num >= DRS_COUNT);
    return s_state[a_drs_num].stage;
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
            if( g_drs_flags & (0x1 << n) ){
                l_ret = s_run(n, a_cal_flags, a_params);
                if(l_ret != 0)
                    break;
            }
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
drs_calibrate_state_t* drs_calibrate_get_state(int a_drs_num)
{
    if(a_drs_num >= DRS_COUNT){
        log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
        return NULL;
    }
    drs_calibrate_state_t * l_ret = DAP_NEW_Z(drs_calibrate_state_t);
    l_ret->is_running = s_state[a_drs_num].is_running;
    l_ret->progress = s_state[a_drs_num].progress;
    l_ret->thread_id = s_state[a_drs_num].thread_id;
    l_ret->ts_start = s_state[a_drs_num].ts_start;
    l_ret->ts_end = s_state[a_drs_num].ts_end;
    l_ret->cal = &s_state[a_drs_num];
    l_ret->stage = s_state[a_drs_num].stage;
    return l_ret;
}

/**
 * @brief drs_calibrate_progress
 * @param a_drs_num
 * @return
 */
int drs_calibrate_progress(int a_drs_num)
{
    if(a_drs_num < 0){
        log_it(L_ERROR, "-1 is not allowed for drs_calibrate_progress() call");
        return -1;
    }
    if(a_drs_num >= DRS_COUNT){
        log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
        return -1;
    }
    drs_calibrate_t * l_cal = &s_state[a_drs_num];
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
    if(a_drs_num < 0){
        log_it(L_ERROR, "-1 is not allowed for drs_calibrate_is_running() call");
        return -1;
    }
    if(a_drs_num >= DRS_COUNT){
        log_it(L_ERROR, "Too big DRS number %d, should be smaller than %d",a_drs_num, DRS_COUNT);
        return -1;
    }
    drs_calibrate_t * l_cal = &s_state[a_drs_num];
    if (l_cal == NULL)
        return -2;

    bool l_ret;
    pthread_rwlock_rdlock(&l_cal ->rwlock);
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
    if(a_drs_num < 0){
        log_it(L_ERROR, "-1 is not allowed for drs_calibrate_abort() call");
        return -1;
    }
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
 * @brief drs_cal_get_x
 * @param a_drs
 * @param a_x
 * @param a_flags
 * @return
 */
int drs_cal_get_x(drs_t * a_drs, double * a_x, int a_flags)
{
    assert(a_x);
    assert(a_drs);

    for (unsigned n =0; n <DRS_CELLS_COUNT_CHANNEL; n++){
        a_x[n] = n;
    }

    if(a_flags & DRS_CAL_APPLY_X_TIME_LOCAL) {
        drs_cal_time_local_apply(a_drs, a_x, a_x);
    }
    if(a_flags & DRS_CAL_APPLY_X_TIME_GLOBAL) {
        drs_cal_time_global_apply(a_drs, a_x,a_x);
    }
    if(a_flags & DRS_CAL_APPLY_PHYS) {
        do_on_array(a_x,DRS_CELLS_COUNT_CHANNEL,s_x_to_real);
    }

    return 0;
}

/**
 * @brief drs_cal_x_produce
 * @param a_drs
 * @param a_flags
 */
double * drs_cal_x_produce(drs_t * a_drs, int a_flags)
{
    double * l_ret = DAP_NEW_SIZE(double, DRS_CELLS_COUNT_CHANNEL*sizeof(double) );
    int l_err;
    if( (l_err = drs_cal_get_x (a_drs,l_ret, a_flags))!= 0){
        DAP_DELETE(l_ret);
        log_it(L_ERROR, "Can't fill X array, code %d", l_err );
        return NULL;
    }
    return l_ret;
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

int drs_cal_get_y(drs_t * a_drs,double * a_y,unsigned a_page, int a_flags_get, int a_flags_apply)
{
    unsigned short l_buf[DRS_CELLS_COUNT];
    int l_ret;
    if ( (l_ret = drs_data_get_page(a_drs,a_flags_get,a_page,l_buf,sizeof (l_buf) )) != 0 ){
        log_it(L_ERROR,"Can't get data from DRS, code %d", l_ret);
        return l_ret;
    }
    drs_cal_y_apply(a_drs,l_buf,a_y,a_flags_apply);
    return 0;
}

/**
 * @brief drs_cal_y_apply
 * @param a_drs
 * @param a_in
 * @param a_out
 * @param a_flags
 */
void drs_cal_y_apply(drs_t * a_drs, unsigned short *a_in,double *a_out, int a_flags)
{
    debug_if(s_debug_more,L_INFO, "drs_cal_y_apply() a_flags = 0x%08X", a_flags);
    unsigned int l_ch_id,l_cell_id,koefIndex;

    // Если мы сейчас в режиме 9ого канала, то автоматически взводим этот флаг
    unsigned l_mode = drs_get_mode(a_drs->id);
    bool l_ch_9_mode =  (l_mode == DRS_MODE_CAL_TIME) || (a_flags & DRS_CAL_APPLY_CH9_ONLY);

    unsigned l_cells_proc_count =  l_ch_9_mode ? DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL;
    //double * l_bi = s_bi; // a_drs->coeffs.b
    //double * l_ki = s_ki; // a_drs->coeffs.k
    //double  **l_bi = a_drs->coeffs.b;
    //double  **l_ki = a_drs->coeffs.k;
    //double average[4];
    //getAverageInt(average,buffer,DRS_CELLS_COUNT_CHANNEL,DRS_CHANNELS_COUNT);

    if (l_ch_9_mode
          &&   ! (a_flags& DRS_CAL_APPLY_NO_ROTATE )
          &&   ! (a_flags& DRS_CAL_APPLY_ROTATE_GLOBAL )
          &&   ! (a_flags& DRS_CAL_APPLY_ROTATE_BANK )){
        a_flags |= DRS_CAL_APPLY_NO_ROTATE;
    }

    bool l_need_to_rotate =  ( ( (! (a_flags & DRS_CAL_APPLY_ROTATE_GLOBAL))
                                 && (!(a_flags & DRS_CAL_APPLY_ROTATE_BANK)) &&
                                 (!(a_flags & DRS_CAL_APPLY_NO_ROTATE))) ||
        (a_flags & DRS_CAL_APPLY_ROTATE_GLOBAL || a_flags & DRS_CAL_APPLY_ROTATE_BANK ) );// &&
                              //( !(a_flags & DRS_CAL_APPLY_Y_CELLS) );

    //l_need_to_rotate = true;


    //double * l_out = l_need_to_rotate ?
    //      DAP_NEW_STACK_SIZE(double, DRS_CELLS_COUNT * sizeof (double)) : a_out;


    for(l_ch_id=0; l_ch_id<DRS_CHANNELS_COUNT;l_ch_id++){
        if(l_ch_9_mode)
            l_ch_id = DRS_CHANNEL_9;

        for(l_cell_id=0; l_cell_id<l_cells_proc_count;l_cell_id++){
            size_t l_cell_id_masked = l_ch_9_mode ? l_cell_id :
                                                                        l_cell_id&3072 ;
            unsigned l_inout_id = l_cell_id * DRS_CHANNELS_COUNT + l_ch_id;

            koefIndex=  l_ch_9_mode ?
                (a_drs->shift_bank + l_cell_id ) & 1023
                : (a_drs->shift_bank + (l_cell_id&1023) ) & 1023 ;

            a_out[l_inout_id] = a_in[l_inout_id];

            if((a_flags & DRS_CAL_APPLY_Y_CELLS)!=0){
                double l_bi, l_ki;
                if (l_ch_9_mode){
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
                a_out[l_inout_id] -=  a_drs->coeffs.chanB[l_ch_id] +  (DRS_ADC_TOP_LEVEL/2.0 ) * (a_drs->coeffs.chanK[l_ch_id] );
                //l_out[l_inout_id] =  (l_out[l_inout_id] - a_drs->coeffs.chanB[l_ch_id])/( a_drs->coeffs.chanK[l_ch_id] )+  (DRS_ADC_TOP_LEVEL/2.0 );
            }

            //if((key&2)!=0)
            //{
            //  dBuf[k*chanalCount+j]-=coef->chanB[j]+coef->chanK[j]*average[j];
            //}


            if((a_flags & DRS_CAL_APPLY_PHYS)!=0){
                a_out[l_inout_id]= (a_out[l_inout_id] + g_ini->fastadc.adc_offsets[a_drs->id* DRS_CHANNELS_COUNT+ l_ch_id])
                    * g_ini->fastadc.adc_gains[a_drs->id* DRS_CHANNELS_COUNT+ l_ch_id];

                a_out[l_inout_id] *= -1.0; // Инвертируем шкалу, так как так оказалось надо
            }

        }
        if(l_ch_9_mode)
            break;
    }

    // Разворачиваем всё вместе

    if ( l_need_to_rotate ){
        if (a_flags & DRS_CAL_APPLY_ROTATE_BANK){
            drs_data_rotate_bank(a_drs, a_out, a_out, DRS_CELLS_COUNT * sizeof (double), sizeof(double));
        }
        if (a_flags & DRS_CAL_APPLY_ROTATE_GLOBAL){
            drs_data_rotate_global(a_drs, a_out, a_out, DRS_CELLS_COUNT * sizeof (double), sizeof(double));
        }

        if(    ((! (a_flags & DRS_CAL_APPLY_ROTATE_GLOBAL))  && (!(a_flags & DRS_CAL_APPLY_ROTATE_BANK)) )  )  {
            drs_data_rotate_bank(a_drs, a_out, a_out, DRS_CELLS_COUNT * sizeof (double), sizeof(double));
            drs_data_rotate_global(a_drs, a_out, a_out, DRS_CELLS_COUNT * sizeof (double), sizeof(double));
        }
    }

    if ( !(a_flags & DRS_CAL_APPLY_NO_CUT) ){
        size_t l_zero_count = DRS_DATA_CUT_LENGTH * DRS_CHANNELS_COUNT;
        if (g_drs_data_cut_from_begin){
            memmove(a_out, a_out + DRS_CHANNELS_COUNT* g_drs_data_cut_from_begin, (DRS_CELLS_COUNT - l_zero_count)* sizeof (double)  );
        }

        if (l_zero_count){
            size_t l_end_count = DRS_CELLS_COUNT - l_zero_count;
            memset(a_out + l_end_count , 0, g_drs_data_cut_from_end * sizeof (double) );
        }
    }

    if( (a_flags& DRS_CAL_APPLY_Y_SPLASHS) || ! (a_flags& DRS_CAL_APPLY_Y_NO_FIX_BAD_CELLS) ){
        drs_cal_amp_remove_splash(a_drs, a_out, drs_cal_get_splash_treshold(), a_flags );
    }

}

/**
 * @brief drs_cal_state_print
 * @param a_reply
 * @param a_cal
 * @param a_limits
 * @param a_flags
 */
void drs_cal_state_print(dap_string_t * a_reply, drs_calibrate_state_t *a_cal, unsigned a_limits, int a_flags)
{
    drs_calibrate_t *l_cal_pvt = a_cal->cal;
    pthread_rwlock_rdlock(&l_cal_pvt->rwlock);
    dap_string_append_printf( a_reply, "Running:     %s\n\n", a_cal->is_running? "yes" : "no" );
    dap_string_append_printf( a_reply, "Progress:    %d%%\n", a_cal->progress );
    dap_string_append_printf( a_reply, "Stage:       %s(0x%08X)\n\n", drs_cal_stage_to_str(a_cal->stage), a_cal->stage );
    if (a_cal->ts_end || true){
        coefficients_t * l_params = &l_cal_pvt->drs->coeffs;
        if ( a_flags & DRS_COEF_SPLASH)
            dap_string_append_array(a_reply, "splash", "%d", l_params->splash, a_limits);


        if ( a_flags & DRS_COEF_DELTA_TIME )
            dap_string_append_array(a_reply, "deltaTimeRef", "%f", l_params->deltaTimeRef, a_limits);

        if ( a_flags & DRS_COEF_CHAN_K )
            dap_string_append_array(a_reply, "chanK", "%lf", l_params->chanK, a_limits);

        if ( a_flags & DRS_COEF_CHAN_B )
            dap_string_append_array(a_reply, "chanB", "%lf", l_params->chanB, a_limits);

        if ( a_flags & DRS_COEF_K_TIME )
            dap_string_append_array(a_reply, "kTime", "%lf", l_params->kTime, a_limits);

        if ( a_flags & DRS_COEF_K ){
            char l_str[128];
            for(unsigned c = 0; c< DRS_CHANNELS_COUNT; c++){
                snprintf(l_str, sizeof(l_str),"k[%u]", c);
                dap_string_append_array(a_reply, l_str, "%lf", l_params->k[c], a_limits);
            }
        }

        if ( a_flags & DRS_COEF_B ){
            char l_str[128];
            for(unsigned c = 0; c< DRS_CHANNELS_COUNT; c++){
                snprintf(l_str, sizeof(l_str),"b[%u]", c);
                dap_string_append_array(a_reply, l_str, "%lf", l_params->b[c], a_limits);
            }
        }

        if ( a_flags & DRS_COEF_K9 )
            dap_string_append_array(a_reply, "k9", "%lf", l_params->k9, a_limits);

        if ( a_flags & DRS_COEF_B9 )
            dap_string_append_array(a_reply, "b9", "%lf", l_params->b9, a_limits);

        dap_string_append_printf( a_reply, "indicator=%d \n\n", l_params->indicator);
        dap_string_append_printf( a_reply, "Got time:    %.3f seconds \n\n",
                                   ((double) (a_cal->ts_end - a_cal->ts_start)) / 1000000000.0 );


    }
    pthread_rwlock_unlock(&l_cal_pvt->rwlock);
}


/**
 * @brief drs_calibrate_params_set_defaults
 * @param a_params
 */
void drs_calibrate_params_set_defaults(drs_calibrate_params_t *a_params)
{
    assert(a_params);
    memset(a_params,0, sizeof(*a_params));
    a_params->ampl.levels[0] = DRS_CAL_BEGIN_DEFAULT;
    a_params->ampl.levels[1] = DRS_CAL_END_DEFAULT;
    a_params->ampl.N = DRS_CAL_N_DEFAULT;
    a_params->ampl.repeats = DRS_CAL_REPEATS;

    a_params->ampl.splash_treshold = DRS_CAL_SPLASH_TRESHOLD_DEFAULT;
    a_params->time_local.min_N = DRS_CAL_MIN_N_DEFAULT;
    a_params->time_local.max_repeats = DRS_CAL_MAX_REPEATS_DEFAULT;
    a_params->time_global.num_cycle = DRS_CAL_NUM_CYCLE_DEFAULT;
}

#define CAL_FILE_MAGIC   0x877BA4E1
typedef struct cal_file_hdr
{
    uint32_t magic;
    int      version;
    uint32_t drs_count;
} DAP_ALIGN_PACKED cal_file_hdr_t;

/**
 * @brief drs_cal_save
 * @return
 */
int drs_cal_save()
{
    cal_file_hdr_t l_hdr = {
        .magic     =  CAL_FILE_MAGIC,
        .version   = 0x1,
        .drs_count = DRS_COUNT
    };
    FILE * f = fopen(s_cal_file_path, "w");
    if ( f == NULL){
        int l_errno = errno;
        char l_strerr[255];
        strerror_r(l_errno, l_strerr, sizeof (l_strerr));
        log_it(L_ERROR, "Can't save file, error string \"%s\" (code %d) ", l_strerr, l_errno );
        return -1;
    }
    fwrite(&l_hdr, sizeof (l_hdr), 1, f);
    for(unsigned i = 0; i < DRS_COUNT; i++)
      fwrite(&g_drs[i].coeffs , sizeof (g_drs[i].coeffs),1, f);
    fclose(f);
    log_it(L_NOTICE, "Saved DRS calibration coeffs to %s", s_cal_file_path);
    return 0;
}

/**
 * @brief drs_cal_load
 * @param a_file_path
 * @return
 */
int drs_cal_load()
{
    cal_file_hdr_t l_hdr = {};

    FILE * f = fopen(s_cal_file_path, "r");
    if ( f == NULL){
        int l_errno = errno;
        char l_strerr[255];
        strerror_r(l_errno, l_strerr, sizeof (l_strerr));
        log_it(L_ERROR, "Can't load from file, error string \"%s\" (code %d) ", l_strerr, l_errno );
        return -1;
    }


    fread(&l_hdr, 1, sizeof (l_hdr), f);
    if (l_hdr.magic != CAL_FILE_MAGIC){
        log_it(L_ERROR, "Wrong file type, expected to see 0x%08X but we have 0x%08X", CAL_FILE_MAGIC, l_hdr.magic);
        return -2;
    }
    if (l_hdr.drs_count != DRS_COUNT){
        log_it(L_ERROR, "Wrong DRS count, need to be %u when there is %u in file", DRS_COUNT, l_hdr.drs_count);
        return -3;
    }
    for(unsigned i = 0; i < DRS_COUNT; i++)
      fread(&g_drs[i].coeffs , sizeof (g_drs[i].coeffs),1, f);
    fclose(f);
    log_it(L_NOTICE, "Loaded calibration params from %s", s_cal_file_path);
    return 0;
}
