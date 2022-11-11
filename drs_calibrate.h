/*
 * drs_calibrate.h
 *
 *  Created on: 8 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include <dap_common.h>
#include "drs.h"
typedef struct{
    pthread_rwlock_t rwlock; // Защищает параметры калибровки

    bool is_running; // Запущен ли прямо сейчас
    unsigned progress; // Progress between 0 and 100

    pthread_t thread_id; // Айди потока
    drs_t * drs; // Объект DRS

    // Сигнализирует завершение калибровки
    pthread_cond_t  finished_cond;
    pthread_mutex_t finished_mutex;
} drs_calibrate_t;

#define DCA_COUNT 8


typedef struct{
    // Амплитудная калибровка
    struct {
        unsigned repeats_count; // (N) количество проходов амплитудной калибровки для каждого уровня цапов

        unsigned levels_count; // (count) количество уровней у амплитудной калибровки levels_count,
                               // для каждого будет N (из repeats_count) проходов,
                               // при нуле будут выполняться два прохода для уровней BegServ и EndServ(о них ниже),
                               // при не нулевом значении, между  BegServ и EndServ будут включены count дополнительных уровней
                               // цапов для амплитудной калибровки
        union{
            double levels[DCA_COUNT+2];
            struct{
                double begin;
                double end;
                double shifts[DCA_COUNT]; // сдвиги цапов
            } DAP_ALIGN_PACKED;
        };
    } ampl;

    // Временная локальная
    struct {
        unsigned min_N; // Min N с клиента, минимальное число набора статистики для каждой ячейки в локальной калибровке
    } time_local;

    // Временная глобальная
    struct {
        unsigned num_cycle; //  numCylce, число проходов в глобальной колибровке
    } time_global;
} drs_calibrate_params_t;


#define DRS_CAL_FLAG_AMPL           0x00000001
#define DRS_CAL_FLAG_TIME_LOCAL     0x00000002
#define DRS_CAL_FLAG_TIME_GLOBAL    0x00000004

int drs_calibrate_init(); // Инициализирует модуль
void drs_calibrate_deinit(); // Денициализирует модуль

int drs_calibrate_run(int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params );
int drs_calibrate_wait_for_finished(int a_drs_num, int a_wait_msec);

bool drs_calibrate_is_running(int a_drs_num);
int drs_calibrate_progress(int a_drs_num);

drs_calibrate_t* drs_calibrate_get_state(int a_drs_num);

int drs_calibrate_abort(int a_drs_num);
