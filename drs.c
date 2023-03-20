/*
 * dac_ops.h
 *
 *  Created on: 21 October
 *      Author: Dmitry Gerasimov
 */
#include <assert.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include <dap_common.h>
#include <dap_config.h>
#include <dap_file_utils.h>
#include <dap_timerfd.h>

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
const char s_drs_check_file[]="/tmp/drs_init";


parameter_t * g_ini = NULL;
drs_dac_ch_params_t g_ini_ch9;

drs_t g_drs[DRS_COUNT]={
    [0]={
        .id = 0
    },
    [1]={
        .id = 1
    }
};

static const double c_freq_DRS[]= {
  [DRS_FREQ_1GHz] = 1.024,
  [DRS_FREQ_2GHz] = 2.048,
  [DRS_FREQ_3GHz] = 3.072,
  [DRS_FREQ_4GHz] = 4.096,
  [DRS_FREQ_5GHz]=  4.915200};

enum drs_freq g_current_freq=DRS_FREQ_5GHz;

static bool s_init_on_start_timer_callback(void* a_arg); // Init on start timeout callback



int drs_init()
{
    // Инициализация DRS
    init_mem();

    g_ini = DAP_NEW_Z(parameter_t);
    drs_ini_load("/media/card/config.ini", g_ini );

    drs_set_freq(g_current_freq);

    // Инициализация параметров DRS по таймеру
    log_it(L_NOTICE,"DRS config and memory are initialized");

    dap_timerfd_start_on_worker(  dap_events_worker_get_auto(),  g_ini->init_on_start_timer_ms,
                                       s_init_on_start_timer_callback, g_ini);
    return 0;
}

/**
 * @brief s_init_on_start_timer_callback
 * @details Callback for timer. If return true,
 *          it will be called after next timeout
 * @param a_arg
 * @return
 */
static bool s_init_on_start_timer_callback(void* a_arg)
{
    UNUSED(a_arg);
    if( ! drs_get_inited() ){
        log_it(L_INFO,"Timeout for init on start passed, initializing DRS...");
        drs_cmd_init();
    }else{
        log_it(L_DEBUG,"DRS is already initialized so lets just pass this stage");
    }
    return false;
}


/**
 * @brief drs_init
 * @param a_params
 */
int drs_cmd_init()
{
    if( drs_get_inited() ){
        log_it(L_WARNING, "Already initialized");
        return -1;
    }
    memw(0xFFC25080,0x3fff); //инициализация работы с SDRAM

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

    // Touch file
    FILE * f = fopen(s_drs_check_file,"w");
    fclose(f);

    log_it(L_NOTICE, "DRS settings are implemented");
    return 0;
}

void drs_set_freq(enum drs_freq a_freq)
{
    g_current_freq = a_freq;
    write_reg(0x4, 1);//select frequency (0 - external, 1 - internal
    write_reg(30,   freqREG[g_current_freq]);//select ref frequency
}

double drs_get_freq_value(enum drs_freq a_freq)
{
    return c_freq_DRS[g_current_freq];
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
    a_prm->init_on_start_timer_ms                  = dap_config_get_item_uint32_default(l_cfg,"COMMON","init_on_start_timer",1000);
    a_prm->fastadc.ROFS1 			= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "ROFS1", 35000 );
    a_prm->fastadc.OFS1				= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "OFS1", 30000 );
    a_prm->fastadc.ROFS2 			= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "ROFS2", 35000 );
    a_prm->fastadc.OFS2				= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "OFS2", 30000 );
    a_prm->fastadc.CLK_PHASE		= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "CLK_PHASE", 40 );
    for (t=0;t<DRS_DCA_COUNT_ALL ;t++){
        sDAC_offset[strlen(sDAC_offset)-1]=t+49;
        a_prm->fastadc.dac_offsets[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sDAC_offset, 0.0);
    }

    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        sDAC_gain[strlen(sDAC_gain)-1]=t+49;
        a_prm->fastadc.dac_gains[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sDAC_gain, 1.0);
    }
    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        sADC_offset[strlen(sADC_offset)-1]=t+49;
        a_prm->fastadc.adc_offsets[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sADC_offset, 0.0);
    }
    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        sADC_gain[strlen(sADC_gain)-1]=t+49;
        a_prm->fastadc.adc_gains[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sADC_gain, 1.0);
    }

    // Для 9 канала
    sDAC_offset[strlen(sDAC_offset)-1]=9 + 49;
    sDAC_gain[strlen(sDAC_offset)-1]=9 + 49;
    g_ini_ch9.offset  = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sDAC_offset, a_prm->fastadc.dac_offsets[0]);
    g_ini_ch9.gain  = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", sDAC_gain, a_prm->fastadc.dac_gains[0]);

    dap_config_close( l_cfg );
    return 0;
}

static drs_mode_t s_mode[DRS_COUNT] = {0};

/**
 * @brief drs_set_mode
 * @param mode
 */
void drs_set_mode(int a_drs_num, drs_mode_t a_mode)
{
    assert(a_drs_num >= -1 && a_drs_num < DRS_COUNT );
    s_mode[a_drs_num] = a_mode;
    write_reg(DRS_MODE_REG, a_mode);
    usleep(100);
    drs_cmd(a_drs_num, INIT_DRS);
}

/**
 * @brief drs_get_mode
 * @param a_drs_num
 * @return
 */
drs_mode_t drs_get_mode(int a_drs_num)
{
    return s_mode[a_drs_num];
}

/**
 * @brief drs_set_dac_input_shift
 * @param addrShift
 * @param value
 */
void drs_dac_shift_input_set(int a_drs_num,unsigned int a_value)
{
    write_reg(0x8+a_drs_num,a_value);
    usleep(100);
}

/**
 * @brief drs_dac_shift_input_set_ch9
 * @param a_value
 */
void drs_dac_shift_input_set_ch9(unsigned int a_value)
{
    write_reg(DRS_REG_DATA_DAC_CH9 ,a_value);
    usleep(100);
}

/**
 * @brief drs_dac_shift_input_get
 * @param a_drs_num
 */
unsigned drs_dac_shift_input_get(int a_drs_num)
{
    return read_reg(0x8+a_drs_num);
}

/**
 * @brief drs_dac_shift_input_get_ch9
 */
unsigned drs_dac_shift_input_get_ch9()
{
    return read_reg(DRS_REG_DATA_DAC_CH9);
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
void drs_dac_shift_input_set_all(int a_drs_num, unsigned short *shiftValue)//fix
{
    drs_dac_shift_input_set(a_drs_num,((shiftValue[0]<<16)&0xFFFF0000)|shiftValue[1]);
}

/**
 * double *shiftDAC		сдвиги с фронтпанели;
 * float *DAC_gain		массив из ini
 * float *DAC_offset	массив из ini
 */
void drs_dac_shift_set_all(int a_drs_num, double *shiftDAC,float *DAC_gain,float *DAC_offset)//fix
{
    int i;
    unsigned short shiftDACValues[DRS_CHANNELS_COUNT];
    assert(shiftDAC);
    assert(DAC_gain);
    assert(DAC_offset);
    for(i=0;i<DRS_CHANNELS_COUNT;i++) {
        shiftDACValues[i]= fabs((shiftDAC[i]+ 0.5)*16384.0) ;
        shiftDACValues[i]=(shiftDACValues[i]*DAC_gain[i]+DAC_offset[i]);
        log_it(L_DEBUG, "shiftDAC[%d]=%f\tshiftDACValues[%d]=%d",i,shiftDAC[i],i,shiftDACValues[i]);
    }
    drs_dac_shift_input_set_all(a_drs_num, shiftDACValues);
    drs_dac_set(1);
    usleep(60);
}

/**
 * @brief drs_dac_shift_set_ch9
 * @param shiftDAC
 * @param DAC_gain
 * @param DAC_offset
 */
void drs_dac_shift_set_ch9(double a_shift,float a_gain,float a_offset)
{
    unsigned short l_shift_DAC_value;
    l_shift_DAC_value= fabs((a_shift+ 0.5)*16384.0) ;
    l_shift_DAC_value=(l_shift_DAC_value*a_gain +a_offset );
    log_it(L_DEBUG, "Set CH9 DAC shift: a_shift_DAC=%f\tl_shift_DAC_value=%d",a_shift,l_shift_DAC_value);
    drs_dac_shift_input_set_ch9( l_shift_DAC_value);
    drs_dac_set(1);
    usleep(60);
}

/**
 * @brief dap_get_inited
 * @return
 */
bool drs_get_inited()
{
    return dap_file_test(s_drs_check_file);
}
