/*
 * dac_ops.h
 *
 *  Created on: 21 October
 *      Author: Dmitry Gerasimov
 */
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>


#include "drs.h"
#include "minIni.h"
#include "calibrate.h"
#include "commands.h"
#include "data_operations.h"

#define inipath "/media/card/config.ini"
#define DEBUG
#define SERVER_NAME "test_server"
#define PORT 3000
#define MAX_SLOWBLOCK_SIZE 1024*1024
#define SIZE_BUF_IN 128
#define MAX_PAGE_COUNT 1000
#define SIZE_FAST MAX_PAGE_COUNT*1024*8*4*4
#define SIZE_NET_PACKAGE 1024//0x100000 // 0x8000 = 32k
#define POLLDELAY 1000 //ns
#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2
unsigned short mas[0x4000];
unsigned short tmas[MAX_SLOW_ADC_SIZE_IN_BYTE+8192*2];// + shift size
unsigned short tmasFast[SIZE_FAST];
unsigned char buf_in[SIZE_BUF_IN];
unsigned int *cmd_data=(uint32_t *)&buf_in[0];//(unsigned int *)
struct sockaddr_in name, cname;
struct timeval tv = { 0, POLLDELAY };

const unsigned int freqREG[]= {480, 240, 160, 120, 100};


parameter_t * g_ini = NULL;

/**
 * @brief drs_init
 * @param prm
 */
void drs_init(parameter_t *a_params)
{
    write_reg(0x4, 1);//select frequency (0 - external, 1 - internal
    write_reg(30, freqREG[g_current_freq]);//select ref frequency

    write_reg(6, a_params->fastadc.CLK_PHASE);//clk_phase
    printf("initialization\tprm->fastadc.OFS1=%u\tprm->fastadc.ROFS1=%u\n",a_params->fastadc.OFS1,a_params->fastadc.ROFS1);
    printf("              \tprm->fastadc.OFS2=%u\tprm->fastadc.ROFS2=%u\n",a_params->fastadc.OFS2,a_params->fastadc.ROFS2);
    printf("              \tprm->fastadc.CLK_PHASE=%u\n", a_params->fastadc.CLK_PHASE);
    usleep(3);
    write_reg(10,((a_params->fastadc.OFS1<<16)&0xffff0000)|a_params->fastadc.ROFS1);// OFS&ROFS
    usleep(3);
    write_reg(11,((0<<16)&0xffff0000)|30000);// DSPEED&BIAS
    usleep(3);
    write_reg(12,((a_params->fastadc.OFS2<<16)&0xffff0000)|a_params->fastadc.ROFS2);// OFS&ROFS
    usleep(3);
    write_reg(13,((0<<16)&0xffff0000)|30000);// DSPEED&BIAS
    usleep(3);
    setDAC(1);
//	write_reg(0x0,1<<3|0<<2|0<<1|0);//Start_DRS Reset_DRS Stop_DRS Soft reset

}

/**
 * @brief Загружает даные из ini файла и сохраняет в параметры
 * @param inifile
 * @param prm
 */
void drs_ini_load(const char *inifile, parameter_t *prm)
{
    char sDAC_gain[]="DAC_gain_X";
    char sADC_offset[]="ADC_offset_X";
    char sADC_gain[]="ADC_gain_X";
    char sDAC_offset[]="DAC_offset_X";
    unsigned char t;
  //  char IP[16];
  //  long n;
    /* string reading */
  //  n = ini_gets("COMMON", "host", "dummy", IP, sizearray(IP), inifile);
  //  printf("Host = %s\n", IP);
  //  n = ini_gets("COMMON", "firmware", "dummy", prm->firmware_path, sizearray(prm->firmware_path), inifile);
    prm->fastadc.ROFS1 			= ini_getl("FASTADC_SETTINGS", "ROFS1", 35000, inifile);
    prm->fastadc.OFS1				= ini_getl("FASTADC_SETTINGS", "OFS1", 30000, inifile);
    prm->fastadc.ROFS2 			= ini_getl("FASTADC_SETTINGS", "ROFS2", 35000, inifile);
    prm->fastadc.OFS2				= ini_getl("FASTADC_SETTINGS", "OFS2", 30000, inifile);
    prm->fastadc.CLK_PHASE		= ini_getl("FASTADC_SETTINGS", "CLK_PHASE", 40, inifile);
    for (t=0;t<4;t++)
    {
     sDAC_offset[strlen(sDAC_offset)-1]=t+49;
     prm->fastadc.dac_offsets[t] = ini_getf("FASTADC_SETTINGS", sDAC_offset, 0.0, inifile);
    }
    for (t=0;t<4;t++)
    {
     sDAC_gain[strlen(sDAC_gain)-1]=t+49;
     prm->fastadc.dac_gains[t] = ini_getf("FASTADC_SETTINGS", sDAC_gain, 1.0, inifile);
    }
    for (t=0;t<4;t++)
    {
     sADC_offset[strlen(sADC_offset)-1]=t+49;
     prm->fastadc.adc_offsets[t] = ini_getf("FASTADC_SETTINGS", sADC_offset, 0.0, inifile);
    }
    for (t=0;t<4;t++)
    {
     sADC_gain[strlen(sADC_gain)-1]=t+49;
     prm->fastadc.adc_gains[t] = ini_getf("FASTADC_SETTINGS", sADC_gain, 1.0, inifile);
    }
}

/**
 * @brief Сохраняет параметры в ini файл
 * @param inifile
 * @param prm
 */
void drs_ini_save(const char *inifile, parameter_t *prm)
{
    char sDAC_offset[]="DAC_offset_X";
    char sDAC_gain[]="DAC_gain_X";
    char sADC_offset[]="ADC_offset_X";
    char sADC_gain[]="ADC_gain_X";
    unsigned char t;
  //  char IP[16];
  //  long n;
    /* string reading */
  //  n = ini_puts("COMMON", "host", IP, inifile);
  //  printf("Host = %s\n", IP);
  //  ini_puts("COMMON", "firmware", prm.firmware_path, inifile);
    ini_putl("FASTADC_SETTINGS", "ROFS1", prm->fastadc.ROFS1, inifile);
    ini_putl("FASTADC_SETTINGS", "OFS1", prm->fastadc.OFS1, inifile);
    ini_putl("FASTADC_SETTINGS", "ROFS2", prm->fastadc.ROFS2, inifile);
    ini_putl("FASTADC_SETTINGS", "OFS2", prm->fastadc.OFS2, inifile);
    ini_putl("FASTADC_SETTINGS", "CLK_PHASE", prm->fastadc.CLK_PHASE, inifile);
    for (t=0;t<4;t++)
    {
     sDAC_offset[strlen(sDAC_offset)-1]=t+49;
     ini_putf("FASTADC_SETTINGS", sDAC_offset, prm->fastadc.dac_offsets[t], inifile);
    }
    for (t=0;t<4;t++)
    {
     sDAC_gain[strlen(sDAC_gain)-1]=t+49;
     ini_putf("FASTADC_SETTINGS", sDAC_gain, prm->fastadc.dac_gains[t], inifile);
    }
    for (t=0;t<4;t++)
    {
     sADC_offset[strlen(sADC_offset)-1]=t+49;
     ini_putf("FASTADC_SETTINGS", sADC_offset, prm->fastadc.adc_offsets[t], inifile);
    }
    for (t=0;t<4;t++)
    {
     sADC_gain[strlen(sADC_gain)-1]=t+49;
     ini_putf("FASTADC_SETTINGS", sADC_gain, prm->fastadc.adc_gains[t], inifile);
    }
}
