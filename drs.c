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
#include "drs_cli.h"

#include "commands.h"
#include "data_operations.h"

#define MAX_SLOWBLOCK_SIZE 1024*1024
#define SIZE_BUF_IN 128
#define MAX_PAGE_COUNT 1000
#define SIZE_NET_PACKAGE 1024//0x100000 // 0x8000 = 32k
#define POLLDELAY 1000 //ns
#define MAX_SLOW_ADC_CHAN_SIZE 0x800000
#define MAX_SLOW_ADC_SIZE_IN_BYTE MAX_SLOW_ADC_CHAN_SIZE*8*2

#define PAGE_SIZE 8192
#define HPS2FPGA_BRIDGE_BASE	0xC0000000 //данные быстрых АЦП
#define LWHPS2FPGA_BRIDGE_BASE	0xff200000 //управление
#define SHIFT_DRS1	0x2FD00000
#define SHIFT_DRS2	0x3FD00000

#define MAP_SIZE           (4096)
#define MAP_MASK           (MAP_SIZE-1)

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
enum drs_freq g_current_freq=DRS_FREQ_5GHz;
void *data_map_drs1, *data_map_drs2, *data_map_shift_drs1, *data_map_shift_drs2, *data_map;

int g_drs_flags = 0;

static const unsigned int freqREG[]= {480, 240, 160, 120, 100};
static const char s_drs_check_file[]="/tmp/drs_init";

static const double c_freq_DRS[]= {
  [DRS_FREQ_1GHz] = 1.024,
  [DRS_FREQ_2GHz] = 2.048,
  [DRS_FREQ_3GHz] = 3.072,
  [DRS_FREQ_4GHz] = 4.096,
  [DRS_FREQ_5GHz]=  4.915200};


static int fd;
static volatile unsigned int *control_mem;
static void *control_map;

static bool s_init_on_start_timer_callback(void* a_arg); // Init on start timeout callback
static int s_init_mem(void);
static uint32_t s_memr(off_t byte_addr);
static void s_memw(off_t byte_addr, uint32_t data);
static void s_post_init();


static bool s_debug_more = false;

static bool s_initalized = false;
/**
 * @brief drs_init
 * @param a_drs_flags
 * @return
 */
int drs_init(int a_drs_flags)
{
    // Проверка на инициализацию
    if( s_initalized ) {
        log_it(L_WARNING, "DRS is already initialized, pls check your code for double call");
        return -1000;
    }
    s_initalized = true;
    g_drs_flags = a_drs_flags;
    // Инициализация DRS
    s_init_mem();

    g_ini = DAP_NEW_Z(parameter_t);
    drs_ini_load("/media/card/config.ini", g_ini );

    drs_set_freq(g_current_freq);

    // Инициализация параметров DRS по таймеру
    log_it(L_NOTICE,"DRS config and memory are initialized");

    dap_timerfd_start_on_worker(  dap_events_worker_get_auto(),  g_ini->init_on_start_timer_ms,
                                       s_init_on_start_timer_callback, g_ini);

    // Инициализация консоли
    if (drs_cli_init() != 0){
        log_it(L_CRITICAL, "Can't init drs cli");
        return -14;
    }
    // Инициализация калибровочных модулей
    drs_calibrate_init();

    return 0;
}

/**
 * @brief s_init_mem
 * @return
 */
static int s_init_mem(void)
{
    int ret = EXIT_FAILURE;
//	unsigned char value;
    off_t control_base = LWHPS2FPGA_BRIDGE_BASE;
    off_t data_base_drs1 = SDRAM_BASE_DRS1;
    off_t data_base_drs2 = SDRAM_BASE_DRS2;
    off_t data_shift_drs1 = SHIFT_DRS1;
    off_t data_shift_drs2 = SHIFT_DRS2;
    off_t data_base_map_offset = MEMORY_BASE;

    /* open the memory device file */
    fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (fd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    /* map the LWHPS2FPGA bridge into process memory */
    control_map = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, control_base);
    if (control_map == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_drs1 = mmap(NULL, SDRAM_SPAN_DRS1, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_base_drs1);
    if (data_map_drs1 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map = mmap(NULL, MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_base_map_offset);
    if (data_map == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_drs2 = mmap(NULL, SDRAM_SPAN_DRS2, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_base_drs2);
    if (data_map_drs2 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_shift_drs1 = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_shift_drs1);
    if (data_map_shift_drs1 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_shift_drs2 = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_shift_drs2);
    if (data_map_shift_drs2 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }
    ret = 0;

cleanup:
    return ret;
}

/**
 * @brief s_deinit_mem
 */
static void s_deinit_mem(void)
{
    if (munmap(control_map, PAGE_SIZE) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map, MEMORY_SIZE) < 0)
    {
        perror("munmap");
        goto cleanup;
    }

    if (munmap(data_map_drs1, SDRAM_SPAN_DRS1) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map_drs2, SDRAM_SPAN_DRS2) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map_shift_drs1, 0x1000) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map_shift_drs2, 0x1000) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
cleanup:
    close(fd);
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
        s_post_init();
    }else{
        log_it(L_DEBUG,"DRS is already initialized so lets just pass this stage");
    }
    return false;
}


/**
 * @brief drs_init
 * @param a_drs_flags
 */
static void s_post_init()
{
    if( drs_get_inited() ){
        log_it(L_WARNING, "Already initialized");
        return;
    }
    s_memw(0xFFC25080,0x3fff); //инициализация работы с SDRAM

    if(g_drs_flags & 0x1){
        set_dma_addr_drs1(0x08000000);  		//    drs_reg_write(0x00000017, 0x8000000);// DRS1
        //set_size_dma_drs1(0x00004000);  		//    drs_reg_write(0x00000019, 0x4000);
    }

    if(g_drs_flags & 0x2){
        set_dma_addr_drs2(0x0c000000);  		//    drs_reg_write(0x00000018, 0xC000000);// DRS2
        //set_size_dma_drs2(0x00004000);  		//    drs_reg_write(0x0000001a, 0x4000);
    }

    if(g_drs_flags & 0x1)
        set_shift_addr_drs1(0x0bf40000);		//    drs_reg_write(0x0000001c, 0xBF40000);
    if(g_drs_flags & 0x2)
        set_shift_addr_drs2(0x0ff40000);		//    drs_reg_write(0x0000001d, 0xFF40000);

    clk_select(INTERNAL_CLK);				//    drs_reg_write(0x00000004, 0x00000001);//select_freq
    clk_select_internal_value(480);			//    drs_reg_write(0x0000001e, 0x00000064);//freqREG[curfreq]);
    clk_phase(40);							//    drs_reg_write(0x00000006, 0x00000028);
    clk_start(1);							//    drs_reg_write(0x00000005, 0x00000001);

    if(g_drs_flags & 0x1)
        set_dac_offs_drs1(30000, 30000);		//    drs_reg_write(0x00000008, 0x83e683e6);
    if(g_drs_flags & 0x2)
        set_dac_offs_drs2(30000, 30000);		//    drs_reg_write(0x00000009, 0x83e683e6);
    start_dac(1);							//    drs_reg_write(0x00000007, 0x00000001);

    //set_dac_rofs_O_ofs_drs1(35000, 30000);
    drs_reg_write(0x0000000a, 0x7d009e98); // чтобы совпадало с логом лабвью

    if(g_drs_flags & 0x1)
        set_dac_speed_bias_drs1(0, 16350);		//    drs_reg_write(0x0000000b, 0x3fde0000);

    //set_dac_rofs_O_ofs_drs2(35000, 30000);	//    drs_reg_write(0x0000000c, 0x7d009e98);
    drs_reg_write(0x0000000c, 0x7d009e98); // чтобы совпадало с логом лабвью

    if(g_drs_flags & 0x2)
        set_dac_speed_bias_drs2(0, 16350);		//    drs_reg_write(0x0000000d, 0x3fde0000);
    set_dac_9ch_ofs(30000);					//    drs_reg_write(0x0000001f, 0x00007530);
    start_dac(1);							//    drs_reg_write(0x00000007, 0x00000001);



    set_gains_drss(32, 32, 32, 32);
    start_amplifier(1);

    if(g_drs_flags & 0x1){
        set_starts_number_drs1(1);
        set_zap_delay_drs1(0);
    }
    if(g_drs_flags & 0x2){
        set_starts_number_drs2(1);
        set_zap_delay_drs2(0);
    }

    set_mode_drss(MODE_SOFT_START);			//    drs_reg_write(0x00000010, 0x00000000);
    if(g_drs_flags & 0x1)
        init_drs1();							//    drs_reg_write(0x0000000e, 0x00000001);
    if(g_drs_flags & 0x2)
        init_drs2();							//    drs_reg_write(0x0000000f, 0x00000001);

    // Start all
    drs_reg_write(0x00000001, 0x0000001);

    // Touch file
    FILE * f = fopen(s_drs_check_file,"w");
    fclose(f);

    // Init all DRS
    //drs_cmd(-1, DRS_CMD_ );

    log_it(L_NOTICE, "DRS settings are implemented");
    if(g_drs_flags & 0x1)
        log_it(L_NOTICE, "--DRS #0 initialized");
    if(g_drs_flags & 0x2)
        log_it(L_NOTICE, "--DRS #1 initialized");
}

void drs_set_freq(enum drs_freq a_freq)
{
    g_current_freq = a_freq;
    drs_reg_write(0x4, 1);//select frequency (0 - external, 1 - internal
    drs_reg_write(30,   freqREG[g_current_freq]);//select ref frequency
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
   drs_calibrate_deinit();

   DAP_DELETE(g_ini);
   g_ini = NULL;

   s_deinit_mem();

}

/**
 * @brief drs_init_old
 * @param prm
 */
void drs_init_old(parameter_t *a_params)
{

    drs_reg_write(6, a_params->fastadc.CLK_PHASE);//clk_phase
    printf("initialization\tprm->fastadc.OFS1=%u\tprm->fastadc.ROFS1=%u\n",a_params->fastadc.OFS1,a_params->fastadc.ROFS1);
    printf("              \tprm->fastadc.OFS2=%u\tprm->fastadc.ROFS2=%u\n",a_params->fastadc.OFS2,a_params->fastadc.ROFS2);
    printf("              \tprm->fastadc.CLK_PHASE=%u\n", a_params->fastadc.CLK_PHASE);
    usleep(3);
    drs_reg_write(10,((a_params->fastadc.OFS1<<16)&0xffff0000)|a_params->fastadc.ROFS1);// OFS&ROFS
    usleep(3);
    drs_reg_write(11,((0<<16)&0xffff0000)|30000);// DSPEED&BIAS
    usleep(3);
    drs_reg_write(12,((a_params->fastadc.OFS2<<16)&0xffff0000)|a_params->fastadc.ROFS2);// OFS&ROFS
    usleep(3);
    drs_reg_write(13,((0<<16)&0xffff0000)|30000);// DSPEED&BIAS
    usleep(3);
    drs_dac_set(1);
//	drs_reg_write(0x0,1<<3|0<<2|0<<1|0);//Start_DRS Reset_DRS Stop_DRS Soft reset

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
    drs_reg_write(DRS_MODE_REG, a_mode);
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
    drs_reg_write(0x8+a_drs_num,a_value);
    usleep(100);
}

/**
 * @brief drs_dac_shift_input_set_ch9
 * @param a_value
 */
void drs_dac_shift_input_set_ch9(unsigned int a_value)
{
    drs_reg_write(DRS_REG_DATA_DAC_CH9 ,a_value);
    usleep(100);
}

/**
 * @brief drs_dac_shift_input_get
 * @param a_drs_num
 */
unsigned drs_dac_shift_input_get(int a_drs_num)
{
    return drs_reg_read(0x8+a_drs_num);
}

/**
 * @brief drs_dac_shift_input_get_ch9
 */
unsigned drs_dac_shift_input_get_ch9()
{
    return drs_reg_read(DRS_REG_DATA_DAC_CH9);
}

/**
 * @brief setDAC
 * @param onAH
 */
void drs_dac_set(unsigned int onAH)//fix
{
    //unsigned int onAH=1,dacSelect=2;
    drs_reg_write(0x07,(onAH&1));
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

void drs_reg_write(unsigned int reg_adr, unsigned int reg_data)
{
    /* get the delay_ctrl peripheral's base address */
    control_mem = (unsigned int *) (control_map + reg_adr*4);
    debug_if(s_debug_more, L_DEBUG, "write: adr=0x%08x (%u), val=0x%08x", reg_adr, reg_adr, reg_data);

    /* write the value */
    *control_mem = reg_data;
    usleep(100);
}

unsigned int drs_reg_read(unsigned int reg_adr)
{
    unsigned int reg_data;
    control_mem = (unsigned int *) (control_map + reg_adr*4);
    reg_data=(unsigned int)control_mem[0];
//    printf("read: adr=0x%08x, val=0x%08x\n\r", reg_adr, reg_data), fflush(stdout);
    usleep(100);
    return(reg_data);
}

static void s_memw(off_t byte_addr, uint32_t data)
{
 void *map_page_addr, *map_byte_addr;
  map_page_addr = mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, byte_addr & ~MAP_MASK );
  if( map_page_addr == MAP_FAILED ) {
    perror( "mmap" );
    return;
  }
  map_byte_addr = map_page_addr + (byte_addr & MAP_MASK);
  *( ( uint32_t *) map_byte_addr ) = data;
  if( munmap( map_page_addr, MAP_SIZE ) ) {
    perror( "munmap" );
    return;
  }
}

static uint32_t s_memr(off_t byte_addr)
{
 void *map_page_addr, *map_byte_addr;
 uint32_t data;
  map_page_addr = mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, byte_addr & ~MAP_MASK );
  if( map_page_addr == MAP_FAILED ) {
    perror( "mmap" );
    return 0;
  }
  map_byte_addr = map_page_addr + (byte_addr & MAP_MASK);
  data = *( ( uint32_t *) map_byte_addr );
  printf( "data = 0x%08x\n", data );
  if( munmap( map_page_addr, MAP_SIZE ) ) {
    perror( "munmap" );
    return 0;
  }
  return (data);
}
