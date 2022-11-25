/*
 * drs_cal.h
 *
 *  Created on: 8 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include <stdatomic.h>
#include <dap_common.h>
#include <dap_time.h>
#include "drs.h"
typedef struct{
    pthread_rwlock_t rwlock;

    bool is_running; // ������� �� ����� ������

    atomic_uint_fast32_t progress; // Progress between 0 and 100

    pthread_t thread_id; // ���� ������
    drs_t * drs; // ������ DRS

    dap_nanotime_t ts_start;
    dap_nanotime_t ts_end;

    // ������������� ���������� ����������
    pthread_cond_t  finished_cond;
    pthread_mutex_t finished_mutex;
} drs_calibrate_t;



typedef struct{
    // ����������� ����������
    struct {
        unsigned repeats; // (N) ���������� �������� ����������� ���������� ��� ������� ������ �����

        unsigned N; // (count) ���������� ������� � ����������� ���������� levels_count,
                               // ��� ������� ����� N (�� repeats_count) ��������,
                               // ��� ���� ����� ����������� ��� ������� ��� ������� BegServ � EndServ(� ��� ����),
                               // ��� �� ������� ��������, �����  BegServ � EndServ ����� �������� count �������������� �������
                               // ����� ��� ����������� ����������
        double levels[DRS_DCA_COUNT_ALL+2];
    } ampl;

    // ��������� ���������
    struct {
        unsigned min_N; // Min N � �������, ����������� ����� ������ ���������� ��� ������ ������ � ��������� ����������
    } time_local;

    // ��������� ����������
    struct {
        unsigned num_cycle; //  numCylce, ����� �������� � ���������� ����������
    } time_global;
} drs_calibrate_params_t;


// ��������� ����������
typedef struct drs_cal_args{
    union{
        struct{
            bool do_amplitude:1;
            bool do_time_local:1;
            bool do_time_global:1;
            unsigned padding:29;
        } DAP_ALIGN_PACKED;
        uint32_t raw; //  ����� ����������, 1 ��� �����������,2 ��������� ���������,3 ���������� ���������
    } keys;
    drs_calibrate_params_t param;
    drs_calibrate_t * cal;
} drs_cal_args_t;

#define DRS_CAL_FLAG_AMPL           0x00000001
#define DRS_CAL_FLAG_TIME_LOCAL     0x00000002
#define DRS_CAL_FLAG_TIME_GLOBAL    0x00000004
#define DRS_CAL_FLAG_TO_REAL        0x00000008

extern int g_current_freq;


int drs_calibrate_init(); // �������������� ������
void drs_calibrate_deinit(); // ��������������� ������

int drs_calibrate_run(int a_drs_num, uint32_t a_cal_flags, drs_calibrate_params_t* a_params );
int drs_calibrate_wait_for_finished(int a_drs_num, int a_wait_msec);

bool drs_calibrate_is_running(int a_drs_num);
int drs_calibrate_progress(int a_drs_num);

drs_calibrate_t* drs_calibrate_get_state(int a_drs_num);

int drs_calibrate_abort(int a_drs_num);

void drs_cal_get_array_x(drs_t * a_drs, double*x, int a_flags);
void drs_cal_get_array_x_old(double*x, unsigned int *shift,coefficients_t *coef,unsigned int key);

