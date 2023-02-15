/*
 * dac_ops.h
 *
 *  Created on: 21 October
 *      Author: Dmitry Gerasimov
 */
#pragma once

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>

#include <dap_common.h>

#include "mem_ops.h"

#define DRS_COUNT 2

// Канал с калибровочной синусоидой для временной калибровки
#define DRS_CHANNEL_9 0

#define DRS_CHANNELS_COUNT 2
#define DRS_CHANNEL_BANK_COUNT 4
#define DRS_BANK_COUNT DRS_CHANNEL_BANK_COUNT * DRS_CHANNELS_COUNT

#define DRS_DATA_MAP_SIZE        0x4000

#define DRS_CELLS_COUNT_BANK      1024
#define DRS_CELLS_COUNT_CHANNEL   4*DRS_CELLS_COUNT_BANK
#define DRS_CELLS_COUNT           DRS_CHANNELS_COUNT*DRS_CELLS_COUNT_CHANNEL
#define DRS_CELLS_COUNT_ALL       DRS_COUNT*DRS_CELLS_COUNT

#define DRS_DCA_COUNT_ALL               4

#define DRS_DAC_COUNT (DRS_DCA_COUNT_ALL/DRS_COUNT)

#define DRS_REG_CMD_DRS_1		14
#define DRS_REG_CMD_DRS_2		15
#define DRS_MODE_REG			16

#define DRS_BASE_NUM_PAGE		19

#define DRS1_NUM_PAGE			19
#define DRS2_NUM_PAGE			20


#define DRS_REG_READY_A                 21
#define DRS_REG_READY_B                 22
#define DRS_REG_CALIB_SIN_ON_CH9        27

#define DRS_REG_DATA_DAC_CH9		31


#define DRS_REG_WAIT_DRS_A              49
#define DRS_REG_WAIT_DRS_B              50

#define DRS_PAGE_ALL_SIZE       (DRS_CELLS_COUNT_ALL * sizeof(unsigned short))

// Задержка после чтения, в микросекундах
#define DRS_PAGE_READ_DELAY             50000

#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2

#define MAX_PAGE_COUNT 1000

#define ADC_FAST_SIZE MAX_PAGE_COUNT*1024*8*8

#define DRS_ADC_TOP_LEVEL 16384.0

typedef struct  {
  float offset;
  float gain;
} drs_dac_ch_params_t;

typedef struct 
{
//  char firmware_path[256];
    uint32_t ROFS1;
    uint32_t OFS1;
    uint32_t ROFS2;
    uint32_t OFS2;
    uint32_t CLK_PHASE;
    float dac_offsets[4];
    float dac_gains[4];
    float adc_offsets[4];
    float adc_gains[4];
} DAP_ALIGN_PACKED fastadc_parameter_t;



typedef struct 
{
  fastadc_parameter_t fastadc;
} DAP_ALIGN_PACKED parameter_t;

typedef struct
{
    double b[DRS_CHANNELS_COUNT][DRS_CELLS_COUNT_CHANNEL];
    double k[DRS_CHANNELS_COUNT][DRS_CELLS_COUNT_CHANNEL];
    double b9[1*DRS_CELLS_COUNT_BANK];
    double k9[1*DRS_CELLS_COUNT_BANK];
    double kTime[1*DRS_CELLS_COUNT_BANK];
    double chanB[DRS_CHANNELS_COUNT];
    double chanK[DRS_CHANNELS_COUNT];
    double deltaTimeRef[DRS_CELLS_COUNT_BANK];
    unsigned int indicator;
    unsigned int splash[DRS_CHANNELS_COUNT];
} DAP_ALIGN_PACKED coefficients_t;

typedef struct{
    short id;
    unsigned int shift;
    coefficients_t coeffs;
} drs_t;

typedef enum {
  DRS_MODE_SOFT_START = 0,
  DRS_MODE_EXT_START  = 1,
  DRS_MODE_PAGE_MODE  = 2,
  DRS_MODE_CAL_AMPL   = 3,
  DRS_MODE_CAL_TIME   = 4,
  DRS_MODE_OFF_INPUTS = 5,
  DRS_MODE_MAX = DRS_MODE_OFF_INPUTS
} drs_mode_t;

enum drs_freq{DRS_FREQ_1GHz,DRS_FREQ_2GHz,DRS_FREQ_3GHz,DRS_FREQ_4GHz,DRS_FREQ_5GHz};

extern enum drs_freq g_current_freq;

extern parameter_t * g_ini;
extern drs_dac_ch_params_t g_ini_ch9;

extern drs_t g_drs[DRS_COUNT];


#define SIZE_FAST MAX_PAGE_COUNT*1024*8*4*4

#define DRS_COEF_SPLASH           0x00000001
#define DRS_COEF_DELTA_TIME       0x00000002
#define DRS_COEF_CHAN_K           0x00000004
#define DRS_COEF_CHAN_B           0x00000008
#define DRS_COEF_K_TIME           0x00000010
#define DRS_COEF_K                0x00000020
#define DRS_COEF_B                0x00000040
#define DRS_COEF_K9               0x00000080
#define DRS_COEF_B9               0x00000100


#define DRS_IDX(ch,n) ((n)*DRS_CHANNELS_COUNT+(ch))
#define DRS_IDX_CAL(n) DRS_IDX(DRS_CHANNEL_9,n)
extern unsigned short tmasFast[SIZE_FAST];

#ifdef __cplusplus
extern "C" {
#endif

int drs_init();
int drs_cmd_init(parameter_t *prm);
bool drs_get_inited();


void drs_deinit();
int drs_ini_save(const char *inifile, parameter_t *prm);
int drs_ini_load(const char *inifile, parameter_t *prm);


void drs_dac_shift_set_all(int a_drs_num, double *shiftDAC,float *DAC_gain,float *DAC_offset);
void drs_dac_shift_set_ch9(double a_shiftDAC,float DAC_gain,float DAC_offset);

void drs_dac_shift_input_set_all(int a_drs_num, unsigned short *shiftValue);

void drs_set_mode(int a_drs_num, drs_mode_t mode);

drs_mode_t drs_get_mode(int a_drs_num);

void drs_dac_shift_input_set(int a_drs_num, unsigned int value);
void drs_dac_shift_input_set_ch9(unsigned int a_value);

unsigned drs_dac_shift_input_get(int a_drs_num);
unsigned drs_dac_shift_input_get_ch9();


void drs_dac_set( unsigned int onAH);
void drs_set_freq(enum drs_freq a_freq);
double drs_get_freq_value(enum drs_freq a_freq);

#ifdef __cplusplus
}
#endif
