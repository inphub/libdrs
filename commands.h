/*
 * commands.h
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: Denis
 */

#pragma once

#include "drs.h"

#define INIT_DRS			1 //EN_WORK
#define START_DRS			2
#define ENABLE_EXT_PULSE	4
#define RESET				8

#define EXTERNAL_CLK	0
#define INTERNAL_CLK	1

#define MODE_SOFT_START	0
#define MODE_EXT_START	1
#define MODE_PAGE		2
#define MODE_AMP_CALIBR	3
#define MODE_TIM_CALIBR	4
#define MODE_CHN_CALIBR	5

void setShiftAllDac(double *shiftDAC,float *DAC_gain,float *DAC_offset);
void setAllDAC(unsigned short *shiftValue);
void setNumPages( unsigned int num);
void setSizeSamples(unsigned int num);

void drs_set_mode( unsigned int mode);

void setDACInputShift(unsigned int addrShift,unsigned int value);
void setDAC(unsigned int onAH);
unsigned int readEnWrite ();
unsigned int readExternalStatus (unsigned int stat);


void set_dma_addr_drs1(unsigned int offs);
void set_size_dma_drs1(unsigned int sz);
void set_dma_addr_drs2(unsigned int offs);
void set_size_dma_drs2(unsigned int sz);
void set_shift_addr_drs1(unsigned int offs);
void set_shift_addr_drs2(unsigned int offs);
void clk_select(unsigned int val);
void clk_phase(unsigned int val);
void clk_select_internal_value(unsigned int val);
void clk_start(unsigned int st);
void set_dac_offs_drs1(unsigned short valch_a, unsigned short valch_b);
void set_dac_offs_drs2(unsigned short valch_c, unsigned short valch_d);
void start_dac(unsigned int st);
void set_dac_rofs_O_ofs_drs1(unsigned short rofs, unsigned short O_ofs);
void set_dac_speed_bias_drs1(unsigned short speed, unsigned short bias);
void set_dac_rofs_O_ofs_drs2(unsigned short rofs, unsigned short O_ofs);
void set_dac_speed_bias_drs2(unsigned short speed, unsigned short bias);
void set_dac_9ch_ofs(unsigned short speed);
void set_gains_drss(unsigned short cha, unsigned short chb, unsigned short chc, unsigned short chd);
void start_amplifier(unsigned int st);
void set_starts_number_drs1(unsigned int num);
void set_zap_delay_drs1(unsigned int delay);
void set_starts_number_drs2(unsigned int num);
void set_zap_delay_drs2(unsigned int delay);
void set_mode_drss(unsigned int mode);
void init_drs1(void);
void init_drs2(void);
void start_drs1(void);
void start_drs2(void);

//// OLD COMMANDS


void pageMode(unsigned int on);
void StartDRSs(unsigned int on);
void setMode(unsigned int mode);
void setTimeCalibrate(unsigned int enable);
void setWorkDRS(unsigned int enable);
void softStartRecorder(unsigned int enable);
void flagEndRead(unsigned int enable);

