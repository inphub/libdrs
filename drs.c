/*
 * dac_ops.h
 *
 *  Created on: 21 October
 *      Author: Dmitry Gerasimov
 */
#include <assert.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <dap_common.h>
#include <dap_config.h>

#define LOG_TAG          "drs"

#include "drs.h"
#include "drs_ops.h"
#include "drs_cal.h"

#include "minIni.h"
#include "calibrate.h"
#include "commands.h"
#include "data_operations.h"
#include "mem_ops.h"

#define inipath "/media/card/config.ini"
#define DEBUG
#define SERVER_NAME "test_server"
#define PORT 3000
#define MAX_SLOWBLOCK_SIZE 1024*1024
#define SIZE_BUF_IN 128
#define MAX_PAGE_COUNT 1000
#define SIZE_NET_PACKAGE 1024//0x100000 // 0x8000 = 32k
#define POLLDELAY 1000 //ns
#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2

unsigned short tmasFast[SIZE_FAST];
const unsigned int freqREG[]= {480, 240, 160, 120, 100};


parameter_t * g_ini = NULL;

drs_t g_drs[DRS_COUNT]={
    [0]={
        .id = 0
    },
    [1]={
        .id = 1
    }
};



int drs_init()
{
    // Инициализация DRS
    init_mem();

    g_ini = DAP_NEW_Z(parameter_t);
    drs_ini_load("/media/card/config.ini", g_ini );
    return 0;
}
/**
 * @brief drs_init
 * @param a_params
 */
int drs_cmd_init(parameter_t *a_params)
{

    set_dma_addr_drs1(0x08000000);  		//    write_reg(0x00000017, 0x8000000);// DRS1
    set_size_dma_drs1(0x00004000);  		//    write_reg(0x00000019, 0x4000);
    set_dma_addr_drs2(0x0c000000);  		//    write_reg(0x00000018, 0xC000000);// DRS2
    set_size_dma_drs2(0x00004000);  		//    write_reg(0x0000001a, 0x4000);
    set_shift_addr_drs1(0x0bf40000);		//    write_reg(0x0000001c, 0xBF40000);
    set_shift_addr_drs2(0x0ff40000);		//    write_reg(0x0000001d, 0xFF40000);

    clk_select(INTERNAL_CLK);				//    write_reg(0x00000004, 0x00000001);//select_freq
    clk_select_internal_value(480);			//    write_reg(0x0000001e, 0x00000064);//freqREG[curfreq]);
    clk_phase(40);							//    write_reg(0x00000006, 0x00000028);
    clk_start(1);							//    write_reg(0x00000005, 0x00000001);
    set_dac_offs_drs1(30000, 30000);		//    write_reg(0x00000008, 0x83e683e6);
    set_dac_offs_drs2(30000, 30000);		//    write_reg(0x00000009, 0x83e683e6);
    start_dac(1);							//    write_reg(0x00000007, 0x00000001);

    //set_dac_rofs_O_ofs_drs1(35000, 30000);
    write_reg(0x0000000a, 0x7d009e98); // чтобы совпадало с логом лабвью

    set_dac_speed_bias_drs1(0, 16350);		//    write_reg(0x0000000b, 0x3fde0000);

    //set_dac_rofs_O_ofs_drs2(35000, 30000);	//    write_reg(0x0000000c, 0x7d009e98);
    write_reg(0x0000000c, 0x7d009e98); // чтобы совпадало с логом лабвью

    set_dac_speed_bias_drs2(0, 16350);		//    write_reg(0x0000000d, 0x3fde0000);
    set_dac_9ch_ofs(30000);					//    write_reg(0x0000001f, 0x00007530);
    start_dac(1);							//    write_reg(0x00000007, 0x00000001);



    set_gains_drss(32, 32, 32, 32);
    start_amplifier(1);

    set_starts_number_drs1(1);
    set_zap_delay_drs1(0);
    set_starts_number_drs2(1);
    set_zap_delay_drs2(0);

    set_mode_drss(MODE_SOFT_START);			//    write_reg(0x00000010, 0x00000000);
    init_drs1();							//    write_reg(0x0000000e, 0x00000001);
    init_drs2();							//    write_reg(0x0000000f, 0x00000001);

    // Start all
    write_reg(0x00000001, 0x0000001);

    return 0;
}

/**
 * @brief drs_deinit
 */
void drs_deinit()
{
   DAP_DELETE(g_ini);
   g_ini = NULL;
}

/**
 * @brief drs_init_old
 * @param prm
 */
void drs_init_old(parameter_t *a_params)
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
    drs_dac_set(1);
//	write_reg(0x0,1<<3|0<<2|0<<1|0);//Start_DRS Reset_DRS Stop_DRS Soft reset

}

/**
 * @brief Загружает даные из ini файла и сохраняет в параметры
 * @param a_ini_path
 * @param a_prm
 */
int drs_ini_load(const char *a_ini_path, parameter_t *a_prm)
{
    char sDAC_gain[]="DAC_gain_X";
    char sADC_offset[]="ADC_offset_X";
    char sADC_gain[]="ADC_gain_X";
    char sDAC_offset[]="DAC_offset_X";
    unsigned char t;
    dap_config_t * l_cfg = dap_config_load(a_ini_path);
    if ( l_cfg == NULL){
        log_it(L_CRITICAL, "Can't load ini file from path %s", a_ini_path);
        return -1 ;
    }
  //  char IP[16];
  //  long n;
    /* string reading */
  //  n = ini_gets("COMMON", "host", "dummy", IP, sizearray(IP), inifile);
  //  printf("Host = %s\n", IP);
  //  n = ini_gets("COMMON", "firmware", "dummy", prm->firmware_path, sizearray(prm->firmware_path), inifile);
    a_prm->fastadc.ROFS1 			= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "ROFS1", 35000 );
    a_prm->fastadc.OFS1				= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "OFS1", 30000 );
    a_prm->fastadc.ROFS2 			= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "ROFS2", 35000 );
    a_prm->fastadc.OFS2				= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "OFS2", 30000 );
    a_prm->fastadc.CLK_PHASE		= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "CLK_PHASE", 40 );
    for (t=0;t<4;t++)
    {
     sDAC_offset[strlen(sDAC_offset)-1]=t+49;
     a_prm->fastadc.dac_offsets[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sDAC_offset, 0.0);
    }
    for (t=0;t<4;t++)
    {
     sDAC_gain[strlen(sDAC_gain)-1]=t+49;
     a_prm->fastadc.dac_gains[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sDAC_gain, 1.0);
    }
    for (t=0;t<4;t++)
    {
     sADC_offset[strlen(sADC_offset)-1]=t+49;
     a_prm->fastadc.adc_offsets[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sADC_offset, 0.0);
    }
    for (t=0;t<4;t++)
    {
     sADC_gain[strlen(sADC_gain)-1]=t+49;
     a_prm->fastadc.adc_gains[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sADC_gain, 1.0);
    }
    dap_config_close( l_cfg );
    return 0;
}

/**
 * @brief Сохраняет параметры в ini файл
 * @param inifile
 * @param prm
 */
int drs_ini_save(const char *inifile, parameter_t *prm)
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
    return 0;
}

/**
 * @brief drs_set_mode
 * @param mode
 */
void drs_mode_set(unsigned int mode)
{
    write_reg(DRS_MODE_REG, mode);
    usleep(100);
    drs_cmd(-1, INIT_DRS);
}

/**
 * @brief drs_set_dac_input_shift
 * @param addrShift
 * @param value
 */
void drs_dac_shift_input_set(unsigned int addrShift,unsigned int value)//fix
{
    write_reg(0x8+addrShift,value);
    usleep(100);
}

/**
 * @brief setDAC
 * @param onAH
 */
void drs_dac_set(unsigned int onAH)//fix
{
    //unsigned int onAH=1,dacSelect=2;
    write_reg(0x07,(onAH&1));
    usleep(200);
}

/**
 * unsigned short *shiftValue 		масиив сдивгов для ЦАП
 */
void drs_dac_shift_input_set_all(unsigned short *shiftValue)//fix
{
    int j;
    for(j=0;j<2;j++)
    {
        drs_dac_shift_input_set(j,((shiftValue[j*2]<<16)&0xFFFF0000)|shiftValue[j*2+1]);
    }
}

/**
 * double *shiftDAC		сдвиги с фронтпанели;
 * float *DAC_gain		массив из ini
 * float *DAC_offset	массив из ini
 */
void drs_dac_shift_set_all(double *shiftDAC,float *DAC_gain,float *DAC_offset)//fix
{
    int i;
    unsigned short shiftDACValues[4];
    assert(shiftDAC);
    assert(DAC_gain);
    assert(DAC_offset);
    for(i=0;i<4;i++)
    {
        shiftDACValues[i]=(shiftDAC[i]*DAC_gain[i]+DAC_offset[i]);
        log_it(L_DEBUG, "shiftDAC[%d]=%f\tshiftDACValues[%d]=%d",i,shiftDAC[i],i,shiftDACValues[i]);
    }
    drs_dac_shift_input_set_all(shiftDACValues);
    drs_dac_set(1);
}

