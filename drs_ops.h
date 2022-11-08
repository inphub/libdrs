/*
 * drs_ops.h
 *
 *  Created on: 3 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */
#pragma once

#include "drs.h"


void drs_start(int a_drs_num);
void drs_set_flag_end_read(int l_drs_num, bool a_enable);
void drs_set_num_pages_all(unsigned int a_num);
void drs_cmd(unsigned int a_drs_num, unsigned int a_cmd);

void drs_set_num_pages(drs_t * a_drs, unsigned int a_num);

bool drs_get_flag_write_ready(int l_drs_num );
