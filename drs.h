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

#define DRS_DATA_MAP_SIZE 0x4000
#define DRS_PAGE_SIZE      8192

#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2


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

extern parameter_t * g_ini;

void drs_init(parameter_t *prm);
void drs_ini_save(const char *inifile, parameter_t *prm);
void drs_ini_load(const char *inifile, parameter_t *prm);


