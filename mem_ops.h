/*
 * memtest.h
 *
 *  Created on: 24 дек. 2014 г.
 *      Author: Zubarev
 */
#pragma once

#include <sys/stat.h>
#include <stdint.h>
#include "drs.h"


#define SDRAM_BASE_DRS1 0x20000000 //536870912
#define SDRAM_SPAN_DRS1 0x0FCFFFFF //253 page = 265289727

#define SDRAM_BASE_DRS2 0x30000000 //805306368
#define SDRAM_SPAN_DRS2 0x0FCFFFFF //253 page = 265289727

extern void *data_map_drs1, *data_map_drs2, *data_map_shift_drs1, *data_map_shift_drs2;

void memw(off_t byte_addr, uint32_t data);
uint32_t memr(off_t byte_addr);
int init_mem(void);
void write_reg(unsigned int reg_adr, unsigned int reg_data);
unsigned int read_reg(unsigned int reg_adr);
void deinit_mem();

