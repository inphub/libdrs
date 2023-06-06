/*
 * drs_cal.h
 *
 *  Created on: 8 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include <stdio.h>
#include <dap_common.h>
#include <dap_time.h>
#include <dap_string.h>
#include "drs.h"

typedef struct{

    // Амплитудная калибровка
    struct {
        unsigned repeats; // (N) количество проходов амплитудной калибровки для каждого уровня цапов

        unsigned N; // (count) количество уровней у амплитудной калибровки levels_count,
                               // для каждого будет N (из repeats_count) проходов,
                               // при нуле будут выполняться два прохода для уровней BegServ и EndServ(о них ниже),
                               // при не нулевом значении, между  BegServ и EndServ будут включены count дополнительных уровней
                               // цапов для амплитудной калибровки
        double splash_gauntlet; // Уровень отсечения всплесков
        double levels[DRS_DCA_COUNT_ALL+2];
    } ampl;

    // Временная локальная
    struct {
        unsigned min_N; // Min N с клиента, минимальное число набора статистики для каждой ячейки в локальной калибровке
        unsigned max_repeats; // Максимальное количество повторов при наборе статистики
    } time_local;

    // Временная глобальная
    struct {
        unsigned num_cycle; //  numCylce, число проходов в глобальной колибровке
    } time_global;
} drs_calibrate_params_t;


typedef struct drs_calibrate drs_calibrate_t;
typedef struct{

    bool is_running; // Запущен ли прямо сейчас

    uint32_t progress; // Progress between 0 and 100

    pthread_t thread_id; // Айди потока

    dap_nanotime_t ts_start;
    dap_nanotime_t ts_end;

    drs_calibrate_t * cal;
} drs_calibrate_state_t;



#define DRS_CAL_FLAG_AMPL           0x00000001
#define DRS_CAL_FLAG_AMPL_INTER     0x00001001
#define DRS_CAL_FLAG_AMPL_SPLASH    0x00002001
#define DRS_CAL_FLAG_TIME_LOCAL     0x00000002
#define DRS_CAL_FLAG_TIME_GLOBAL    0x00000004

// применение калибровки для ячеек
#define DRS_CAL_APPLY_Y_CELLS  BIT(0)
// межканальная калибровка
#define DRS_CAL_APPLY_Y_INTERCHANNEL  BIT(1)
// избавление от всплесков
#define DRS_CAL_APPLY_Y_SPLASHS  BIT(2)

// Временная локальная калибровка
#define DRS_CAL_APPLY_X_TIME_LOCAL  BIT(8)
// Временная глобальная калибровка
#define DRS_CAL_APPLY_X_TIME_GLOBAL  BIT(9)

// Выровнять итоговые результаты
//#define DRS_CAL_APPLY_Y_EQUALIZE  BIT(29)


// приведение к физическим величинам
#define DRS_CAL_APPLY_PHYS  BIT(28)
// Развернуть итоговые результаты
#define DRS_CAL_APPLY_NO_ROTATE  BIT(30)

// Только 9ый канал
#define DRS_CAL_APPLY_CH9_ONLY           BIT(31)

extern struct drs_cal_apply_flags{ const char* brief; const char * descr;} g_drs_cal_apply_to_str[];

#define dap_string_append_array(a_reply, a_name, a_fmt, a_array, a_limits )\
    {\
        size_t l_array_count = (sizeof(a_array))/sizeof(a_array[0]); \
        size_t l_limit = a_limits == 0 || a_limits > l_array_count ? l_array_count  : a_limits; \
        dap_string_append_printf(a_reply,"%s:{",a_name);\
        for (size_t i = 0; i < l_limit ; i++){\
            dap_string_append_printf(a_reply, a_fmt, a_array[i]); \
            if (i != l_limit-1 ) \
                dap_string_append_printf(a_reply, ", "); \
            if ( i && i % 16 == 0 ) \
                dap_string_append_printf(a_reply, "\n"); \
        } \
        dap_string_append_printf(a_reply,"}\n\n");\
    }


#ifdef __cplusplus
extern "C" {
#endif

int drs_calibrate_init(); // Инициализирует модуль
void drs_calibrate_deinit(); // Денициализирует модуль

void drs_calibrate_params_set_defaults(drs_calibrate_params_t *a_params);

int drs_calibrate_run(int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params );
bool drs_calibrate_is_running(int a_drs_num);
int drs_calibrate_progress(int a_drs_num);
drs_calibrate_state_t* drs_calibrate_get_state(int a_drs_num);
int drs_calibrate_wait_for_finished(int a_drs_num, int a_wait_msec);
int drs_calibrate_abort(int a_drs_num);
int drs_cal_save(const char * a_file_path);
int drs_cal_load(const char * a_file_path);

int drs_cal_get_x(drs_t * a_drs, double * a_x, int a_flags);
int drs_cal_get_y(drs_t * a_drs,double * a_y, unsigned a_page, int a_flags_get, int a_flags_apply);

void drs_cal_set_splash_gauntlet(unsigned a_gauntlet);
unsigned drs_cal_get_splash_gauntlet();

double* drs_cal_x_produce(drs_t * a_drs, int a_flags);
double * drs_cal_y_produce(drs_t * a_drs,int a_flags);

void drs_cal_y_apply(drs_t * a_drs, unsigned short *buffer,double *dBuf, int a_flags);

int drs_cal_y_ch_equalize(drs_t * a_drs, const double *a_y_in, double * a_y_out, double a_avg_level);

void drs_cal_state_print(dap_string_t * a_reply, drs_calibrate_state_t *a_state,unsigned a_limits, int a_flags );


#ifdef __cplusplus
}
#endif
