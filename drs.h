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
#define DRS_CHANNELS_COUNT 2

#define DRS_DATA_MAP_SIZE   0x4000
#define DRS_CELLS_COUNT       8192

#define DRS_DCA_COUNT_ALL               4

#define DRS_DCA_COUNT (DRS_DCA_COUNT_ALL/DRS_COUNT)

#define DRS_BASE_CONTROL_REG            14
#define DRS1_CONTROL_REG		14
#define DRS2_CONTROL_REG		15
#define DRS_MODE_REG			16

#define DRS_BASE_NUM_PAGE		19

#define DRS1_NUM_PAGE			19
#define DRS2_NUM_PAGE			20

#define DRS_CELLS_COUNT_ALL      (DRS_COUNT * DRS_CELLS_COUNT )
#define DRS_PAGE_ALL_SIZE       (DRS_CELLS_COUNT_ALL * sizeof(unsigned short))

#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2

#define MAX_PAGE_COUNT 1000

#define ADC_FAST_SIZE MAX_PAGE_COUNT*1024*8*8


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
    double b[2*4096];
    double k[2*4096];
    double b9[1*1024];
    double k9[1*1024];
    double kTime[1*1024];
    double chanB[2];
    double chanK[2];
    double deltaTimeRef[1*1024];
    unsigned int indicator;
    unsigned int splash[2];
} DAP_ALIGN_PACKED coefficients_t;

typedef struct{
    short id;
    uint32_t shifts [1024];
    unsigned int shift;
    coefficients_t coeffs;
} drs_t;

extern parameter_t * g_ini;
extern drs_t g_drs[DRS_COUNT];

#define SIZE_FAST MAX_PAGE_COUNT*1024*8*4*4

extern unsigned short tmasFast[SIZE_FAST];


int drs_init();
int drs_cmd_init(parameter_t *prm);
void drs_deinit();
int drs_ini_save(const char *inifile, parameter_t *prm);
int drs_ini_load(const char *inifile, parameter_t *prm);


void drs_dac_shift_set_all(double *shiftDAC,float *DAC_gain,float *DAC_offset);
void drs_dac_shift_input_set_all(unsigned short *shiftValue);
void drs_mode_set( unsigned int mode);
void drs_dac_shift_input_set(unsigned int addrShift,unsigned int value);
void drs_dac_set(unsigned int onAH);

