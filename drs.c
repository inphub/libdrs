/*
 * drs.h
 *
 *  Created on: 21 October 2022
 *      Author: Dmitry Gerasimov
 */
#include <assert.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include <pthread.h>


#include <dap_common.h>
#include <dap_config.h>
#include <dap_file_utils.h>
#include <dap_timerfd.h>

#define LOG_TAG          "drs"

#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"
#include "drs_cal.h"
#include "drs_cal_pvt.h"
#include "drs_cli.h"
#include "drs_macs.h"

#include "commands.h"
#include "data_operations.h"

#define MAX_SLOWBLOCK_SIZE 1024*1024
#define SIZE_BUF_IN 128
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
        .id = 0,
        .coeffs = {
            .time_local = {
                .stats_rw = PTHREAD_RWLOCK_INITIALIZER
            }
        }
    },
    [1]={
        .id = 1,
        .coeffs = {
            .time_local = {
                .stats_rw = PTHREAD_RWLOCK_INITIALIZER
            }
        }
    }
};
enum drs_freq g_current_freq=DRS_FREQ_5GHz;
void *data_map_drs1, *data_map_drs2, *g_drs_map_shift_drs1, *g_drs_map_shift_drs2, *data_map;

int g_drs_flags = 0;
va_list s_drs_flags_vars;

unsigned g_drs_data_cut_from_begin = DRS_DATA_CUT_BEGIN_DEFAULT;
unsigned g_drs_data_cut_from_end   = DRS_DATA_CUT_END_DEFAULT;

unsigned short g_drs_gain_default = DRS_GAIN_QUANTS_DEFAULT;

static const unsigned int freqREG[]= {
  [DRS_FREQ_1GHz]=480,
  [DRS_FREQ_2GHz]=240,
  [DRS_FREQ_3GHz]=160,
  [DRS_FREQ_4GHz]=120,
  [DRS_FREQ_5GHz]=100
};
static const char s_drs_check_file[]="/tmp/drs_init";

static int fd;
static volatile unsigned int *control_mem;
static void *control_map;

static bool s_debug_more = false;
static bool s_initalized = false;
static pthread_cond_t s_initalized_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t s_initalized_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned s_dac_shifts_values[DRS_COUNT ] ={0};

static bool s_init_on_start_timer_callback(void* a_arg); // Init on start timeout callback
static int s_init_mem(void);
static uint32_t s_memr(off_t byte_addr);
static void s_memw(off_t byte_addr, uint32_t data);
static void s_hw_init();
static int s_post_init();

static int s_ini_load(const char *a_ini_path, parameter_t *a_prm);

#define MAC_ADDR_SIZE 6
static int s_read_mac(char * a_mac_buf);

/**
 * @brief dap_get_inited
 * @return
 */
static inline bool s_get_inited()
{
    return false;
    if (dap_config_get_item_bool_default(g_config,"drs","check_hw_init", false))
        return dap_file_test(s_drs_check_file);
    else
        return false;
}

/**
 * @brief s_read_mac
 * @param a_mac_buf
 */
static int s_read_mac(char * a_mac_buf)
{
    char * l_ret = NULL;
    char * l_cmd = "cat ";
    int l_ret_code;

    if ( (l_ret_code = exec_with_ret(&l_ret, l_cmd)) != 0 ){
        log_it(L_CRITICAL, "Can't read mac address, return code %d", l_ret_code);
        return l_ret_code;
    }
    return 0;
}
/**
 * @brief drs_init
 * @param a_drs_flags
 * @return
 */
int drs_init(int a_drs_flags,...)
{
    va_list a_vars;
    // Проверка на инициализацию
    if( s_initalized ) {
        log_it(L_WARNING, "DRS is already initialized, pls check your code for double call");
        return -1000;
    }
    s_initalized = true;
    g_drs_flags = a_drs_flags;

    va_start(a_vars, a_drs_flags);
    va_copy(s_drs_flags_vars, a_vars);
    va_end(a_vars);

    // Инициализация DRS
    s_init_mem();

    g_ini = DAP_NEW_Z(parameter_t);


    s_ini_load("/opt/config/config.ini", g_ini );


    s_debug_more = dap_config_get_item_bool_default(g_config,"drs","debug_more", false);


    if(g_drs_flags & DRS_INIT_SET_ONCE_FREQ ||  g_drs_flags & DRS_INIT_SET_ALWAYS_FREQ ){
        g_current_freq = va_arg(s_drs_flags_vars, enum drs_freq );
    }

    if(g_drs_flags & DRS_INIT_SET_ONCE_GAIN_QUANTS_DEFAULT ||  g_drs_flags & DRS_INIT_SET_ALWAYS_GAIN_QUANTS_DEFAULT ){
        g_drs_gain_default = va_arg(s_drs_flags_vars, unsigned );
    }

    if(g_drs_flags & DRS_INIT_SET_DATA_CUT_FROM_BEGIN ){
        g_drs_data_cut_from_begin = va_arg(s_drs_flags_vars, unsigned);
    }

    if(g_drs_flags & DRS_INIT_SET_DATA_CUT_FROM_END )
        g_drs_data_cut_from_end = va_arg(s_drs_flags_vars, unsigned);


    drs_data_init();
    drs_ops_init();
    // Инициализация параметров DRS по таймеру

    log_it(L_NOTICE,"DRS config and memory are initialized");

    pthread_mutex_lock(&s_initalized_mutex);

    dap_timerfd_start_on_worker(  dap_events_worker_get_auto(),  g_ini->init_on_start_timer_ms,
                                   s_init_on_start_timer_callback, g_ini);

    pthread_cond_wait(&s_initalized_cond, &s_initalized_mutex);
    pthread_mutex_unlock(&s_initalized_mutex);

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

    g_drs_map_shift_drs1 = mmap(NULL, DRS_PAGE_COUNT_MAX * sizeof(unsigned) , PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_shift_drs1);
    if (g_drs_map_shift_drs1 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    g_drs_map_shift_drs2 = mmap(NULL, DRS_PAGE_COUNT_MAX * sizeof(unsigned), PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_shift_drs2);
    if (g_drs_map_shift_drs2 == MAP_FAILED) {
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
    if (munmap(g_drs_map_shift_drs1, 0x1000) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(g_drs_map_shift_drs2, 0x1000) < 0)
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
    if( ! s_get_inited() ){
        log_it(L_INFO,"Timeout for init on start passed, initializing DRS...");
        s_hw_init();
    }else{
        log_it(L_DEBUG,"DRS is already initialized so lets just pass this stage");
    }
    s_post_init();

    // Чтобы точно успел ожидающий поток встать на pthread_cond_wait
    pthread_mutex_lock(&s_initalized_mutex);
    pthread_mutex_unlock(&s_initalized_mutex);

    // А вот тут собственно и вызываем
    pthread_cond_broadcast(&s_initalized_cond);


    return false;
}

/**
 * @brief s_post_init
 */
static int s_post_init()
{
    // Инициализация консоли
    if (drs_cli_init() != 0){
        log_it(L_CRITICAL, "Can't init drs cli");
        return -14;
    }
    // Инициализация калибровочных модулей
    drs_calibrate_init();

    double l_shifts[]={0.0,0.0};
    for (unsigned n = 0; n < DRS_COUNT; n++)
        drs_set_dac_offsets_all (n, l_shifts);

    if(g_drs_flags &DRS_INIT_SET_ALWAYS_FREQ )
        drs_set_freq( g_current_freq );

    if(g_drs_flags & DRS_INIT_SET_ALWAYS_GAIN_QUANTS_DEFAULT){
          drs_set_gain_quants(-1, -1, g_drs_gain_default );
    }

    return 0;
}

/**
 * @brief drs_init
 * @param a_drs_flags
 */
static void s_hw_init()
{

    if( s_get_inited() ){
        log_it(L_WARNING, "Already initialized");
        return;
    }
    s_memw(0xFFC25080,0x3fff); //инициализация работы с SDRAM

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0){
        set_dma_addr_drs1(0x08000000);  		//    drs_reg_write(0x00000017, 0x8000000);// DRS1
        //set_size_dma_drs1(0x00004000);  		//    drs_reg_write(0x00000019, 0x4000);
    }

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1){
        set_dma_addr_drs2(0x0c000000);  		//    drs_reg_write(0x00000018, 0xC000000);// DRS2
        //set_size_dma_drs2(0x00004000);  		//    drs_reg_write(0x0000001a, 0x4000);
    }

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0){
        set_shift_addr_drs1(0x0bf40000);		//    drs_reg_write(0x0000001c, 0xBF40000);
    }
    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1){
        set_shift_addr_drs2(0x0ff40000);		//    drs_reg_write(0x0000001d, 0xFF40000);
    }

    drs_reg_write(DRS_REG_SELECT_FREQ, 1);//select frequency (0 - external, 1 - internal

    drs_reg_write(DRS_REG_CLK_PHASE, 40);
    drs_reg_write(DRS_REG_START_S, 1);// write frequency
    usleep(100);

    //if(g_drs_flags &DRS_INIT_SET_ONCE_FREQ  ){
    drs_set_freq( g_current_freq );

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0){
        drs_set_dac_offsets_quants_group(0, g_ini->fastadc.dac_offsets[0],g_ini->fastadc.dac_offsets[1] );
        drs_set_ROFS_n_OFS ( 0,g_ini->fastadc.ROFS[0], g_ini->fastadc.OFS[0]);
        drs_set_dac_speed_bias (0,0, g_ini->fastadc.BIAS[0]);
    }
    log_it(L_DEBUG, "dac_offset[0] = %f",g_ini->fastadc.dac_offsets[0] );

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1){
        drs_set_dac_offsets_quants_group(1, g_ini->fastadc.dac_offsets[2],g_ini->fastadc.dac_offsets[3] );
        drs_set_ROFS_n_OFS ( 0,g_ini->fastadc.ROFS[1], g_ini->fastadc.OFS[1]);
        drs_set_dac_speed_bias (0,0, g_ini->fastadc.BIAS[1]);
    }
    drs_set_dac_sin(g_ini->fastadc.DAC_SIN );					//    drs_reg_write(0x0000001f, 0x00007530);
    drs_start_dac();

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0){
        set_starts_number_drs1(1);
        set_zap_delay_drs1(0);
    }
    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1){
        set_starts_number_drs2(1);
        set_zap_delay_drs2(0);
    }

    //if(g_drs_flags & DRS_INIT_SET_ONCE_GAIN_QUANTS_DEFAULT){
    //      drs_set_gain_quants(-1, -1, g_drs_gain_default );
    //}

    drs_set_gain_quants(-1, -1, g_drs_gain_default );

    set_mode_drss(MODE_SOFT_START);			//    drs_reg_write(0x00000010, 0x00000000);
    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0){
        init_drs1();							//    drs_reg_write(0x0000000e, 0x00000001);
    }
    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1){
        init_drs2();							//    drs_reg_write(0x0000000f, 0x00000001);
    }

    // Start all
    drs_reg_write(0x00000001, 0x0000001);

    drs_reg_write(DRS_REG_AMPL_INIT, 1);

    // Touch file
    FILE * f = fopen(s_drs_check_file,"w");
    fclose(f);

    // Init all DRS
    //drs_cmd(-1, DRS_CMD_ );

    log_it(L_NOTICE, "DRS settings are implemented");
    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0)
        log_it(L_NOTICE, "--DRS #0 initialized");
    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1)
        log_it(L_NOTICE, "--DRS #1 initialized");

}

void drs_set_dac_speed_bias(int a_drs_num, unsigned short a_speed, unsigned short a_bias )
{
#if DRS_COUNT == 2
  static const unsigned c_regs[]={DRS_REG_DAC_0_BIAS, DRS_REG_DAC_1_BIAS};
  if (a_drs_num == -1){
      for(unsigned i = 0; i < ( sizeof (c_regs)/ sizeof(*c_regs) ); i++ )
          drs_reg_write(c_regs[i], (((unsigned int)a_bias)<<16)|((unsigned int)a_speed));

      return;
  }
  if (a_drs_num >= DRS_COUNT || a_drs_num < 0 ){
      log_it(L_ERROR, "Неправильный номер ДРС %u", a_drs_num);
      return;
  }
  drs_reg_write(c_regs[a_drs_num], (((unsigned int)a_bias)<<16)|((unsigned int)a_speed));

#else
#error "Тут автоматически посчитать не выйдет, в какой регистр надо писать BIAS"
#endif
}

/**
 * @brief drs_set_ROFS_n_OFS
 * @param a_drs_num
 * @param a_ROFS
 * @param a_OFS
 */
void drs_set_ROFS_n_OFS(int a_drs_num, unsigned short a_ROFS, unsigned short a_OFS )
{
#if DRS_COUNT == 2
  static const unsigned c_regs[]={DRS_REG_DAC_0_ROFS_n_OFS ,
                                  DRS_REG_DAC_0_ROFS_n_OFS};
  if (a_drs_num == -1){
      for(unsigned i = 0; i < ( sizeof (c_regs)/ sizeof(*c_regs) ); i++ )
          drs_reg_write(c_regs[i], (((unsigned int)a_OFS)<<16)|((unsigned int)a_ROFS));

      return;
  }
  if (a_drs_num >= DRS_COUNT || a_drs_num < 0 ){
      log_it(L_ERROR, "Неправильный номер ДРС %u", a_drs_num);
      return;
  }
  drs_reg_write(c_regs[a_drs_num],(((unsigned int)a_OFS)<<16)|((unsigned int)a_ROFS));

#else
#error "Тут автоматически посчитать не выйдет, в какой регистр надо писать BIAS"
#endif
}


/**
 * @brief drs_set_dac_offset
 * @param a_drs_num
 * @param a_offset
 */
void drs_set_dac_offsets_quants_group(int a_drs_num, unsigned short a_offset_ch0, unsigned short a_offset_ch1)
{
#if DRS_COUNT == 2
    static const unsigned c_regs[]={DRS_REG_DAC_0_OFFSET ,
                                    DRS_REG_DAC_1_OFFSET};
    if (a_drs_num == -1){
        for(unsigned i = 0; i < ( sizeof (c_regs)/ sizeof(*c_regs) ); i++ )
            drs_reg_write(c_regs[i], (((unsigned int)a_offset_ch1)<<16)|((unsigned int)a_offset_ch0));
        return;
    }
    if (a_drs_num >= DRS_COUNT || a_drs_num < 0 ){
        log_it(L_ERROR, "Неправильный номер ДРС %u", a_drs_num);
        return;
    }
    drs_reg_write(c_regs[a_drs_num], (((unsigned int)a_offset_ch1)<<16)|((unsigned int)a_offset_ch0));
#else
#error "Тут автоматически посчитать не выйдет, в какой регистр надо писать BIAS"
#endif

}



/**
 * @brief drs_set_gain_quants
 * @param a_drs_num       Номер ДРС, -1 если для всех
 * @param a_drs_channel   Номер канала, -1 если для всех
 * @param a_gain_q        Гейн в квантах от 0 до 32
 */
void drs_set_gain_quants (int a_drs_num, int a_drs_channel, const unsigned short a_gain_quants)
{
#if DRS_COUNT ==2 && DRS_CHANNELS_COUNT == 2
    unsigned short l_values[4] = {
        g_drs[0].hw.gain[0],
        g_drs[0].hw.gain[1], g_drs[1].hw.gain[0], g_drs[1].hw.gain[1]
    };
    for (unsigned d = 0; d <DRS_COUNT; d++){
        if (a_drs_num != -1)
            d = a_drs_num;

        for (unsigned c=0; c < DRS_CHANNELS_COUNT; c++){
            if( a_drs_channel != -1 )
                c = a_drs_channel;

            // Собственно тут и заполняем массив значений
            l_values[d*DRS_CHANNELS_COUNT + c ] = a_gain_quants;

            if ( a_drs_channel != -1)
                break;
        }

        if (a_drs_num != -1)
            break;
    }

    drs_set_gain_quants_all(l_values);
#else
#error "Нужно переделать работу с регистрами, если общее число каналов не равно 4"
#endif

}

/**
 * @brief drs_set_gain_all
 * @param a_gain
 */
void drs_set_gain_all(const double a_gain[DRS_COUNT * DRS_CHANNELS_COUNT] )
{
    unsigned short l_gain_quants[DRS_COUNT * DRS_CHANNELS_COUNT];
    for (unsigned d = 0; d < DRS_COUNT; d++)
        for (unsigned c = 0; c < DRS_CHANNELS_COUNT; c++){
            unsigned i = d*DRS_CHANNELS_COUNT + c;
            l_gain_quants[i] = drs_gain_db_to_quants(a_gain[i]);
        }

    drs_set_gain_quants_all(l_gain_quants);
}

/**
 * @brief drs_get_gain_quants_all
 * @param a_gain
 */
void drs_get_gain_quants_all(unsigned short a_gain[DRS_COUNT * DRS_CHANNELS_COUNT] )
{
#if DRS_COUNT ==2 && DRS_CHANNELS_COUNT == 2
    a_gain[0]= g_drs[0].hw.gain[0];
    a_gain[1]= g_drs[0].hw.gain[1];
    a_gain[2]= g_drs[1].hw.gain[0];
    a_gain[3]= g_drs[1].hw.gain[1];
#else
#error "Нужно переделать работу с регистрами, если общее число каналов не равно 4"
#endif

}

void drs_set_dac_sin_offset(unsigned short a_offset)
{
    drs_reg_write(DRS_REG_DAC_SIN, a_offset);
}


/**
 * @brief drs_set_gain
 * @details Выставляет значение гейна в Дб для выбранных ДРС и канала, либо для всех сразу
 * @param a_drs_num         Номер ДРС, -1 если для всех
 * @param a_drs_channel     Номер канала, -1 если для всех
 * @param a_gain            значение гейна в Дб от -6 до 26
 */
void drs_set_gain (int a_drs_num, int a_drs_channel, const double a_gain)
{
#if DRS_COUNT ==2 && DRS_CHANNELS_COUNT == 2
    unsigned short l_values[4] = {
        g_drs[0].hw.gain[0],
        g_drs[0].hw.gain[1], g_drs[1].hw.gain[0], g_drs[1].hw.gain[1]
    };
    for (unsigned d = 0; d <DRS_COUNT; d++){
        if (a_drs_num != -1)
            d = a_drs_num;

        for (unsigned c=0; c < DRS_CHANNELS_COUNT; c++){
            if( a_drs_channel != -1 )
                c = a_drs_channel;

            // Собственно тут и заполняем массив значений
            l_values[ d*DRS_CHANNELS_COUNT + c ] = drs_gain_db_to_quants(a_gain);

            if ( a_drs_channel != -1)
                break;
        }
        usleep(3);

        if (a_drs_num != -1)
            break;
    }

    drs_set_gain_quants_all(l_values);
#else
#error "Нужно переделать работу с регистрами, если общее число каналов не равно 4"
#endif

}



/**
 * @brief drs_set_freq
 * @param a_freq
 */
void drs_set_freq(enum drs_freq a_freq)
{
    if(a_freq > DRS_FREQ_MAX){
        log_it(L_ERROR, "Wrong frequency enum %d expected between 0 and %d", a_freq, DRS_FREQ_MAX );
        return;
    }
    g_current_freq = a_freq;
    drs_reg_write(DRS_REG_REF_FREQ,   freqREG[g_current_freq]);//select ref frequency
    drs_reg_write(DRS_REG_START_S, 1);// write frequency
    usleep(100);

    drs_cal_file_path_update();
    drs_cal_load();
    log_it(L_NOTICE, "Set frequency %s", c_freq_str_full[a_freq]);

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
   drs_data_deinit();
   drs_ops_deinit();

   DAP_DELETE(g_ini);
   g_ini = NULL;

   s_deinit_mem();

}

/**
 * @brief Загружает даные из ini файла и сохраняет в параметры
 * @param a_ini_path
 * @param a_prm
 */
static int s_ini_load(const char *a_ini_path, parameter_t *a_prm)
{
    char l_str_DAC_gain[]="DAC_gain_X";
    char l_str_ADC_offset[]="ADC_offset_X";
    char l_str_ADC_gain[]="ADC_gain_X";
    char l_str_DAC_offset[]="DAC_offset_X";
    char l_str_DAC_offset_init_quants[]="DAC_offset_init_quants_X";
    char l_str_DAC_bias[]="BIASX";
    char l_str_KT[]="KT_X";

    unsigned l_dac_init_quants[DRS_COUNT*DRS_CHANNELS_COUNT]={32768,32768};
    unsigned char t;
    dap_config_t * l_cfg = dap_config_load(a_ini_path);
    if ( l_cfg == NULL){
        log_it(L_CRITICAL, "Can't load ini file from path %s", a_ini_path);
        return -1 ;
    }

    // Определяем девайс
    char l_mac_addr[6] = {};
    const char * l_prefix = "/media/card/configs/";
    dap_config_t * l_cfg_device = NULL;
    //if(s_read_mac(l_mac_addr) == 0  ){
    //}else  {
    //    dap_config_load(a_ini_path);
    //}


  //  char IP[16];
  //  long n;
    /* string reading */
  //  n = ini_gets("COMMON", "host", "dummy", IP, sizearray(IP), inifile);
  //  printf("Host = %s\n", IP);
  //  n = ini_gets("COMMON", "firmware", "dummy", prm->firmware_path, sizearray(prm->firmware_path), inifile);
    a_prm->init_on_start_timer_ms                  = dap_config_get_item_uint32_default(l_cfg,"COMMON","init_on_start_timer",1000);
    a_prm->fastadc.ROFS[0] 			= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "ROFS1", 35000 );
    a_prm->fastadc.OFS[0]				= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "OFS1", 30000 );
    a_prm->fastadc.ROFS[1] 			= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "ROFS2", 35000 );
    a_prm->fastadc.OFS[1]				= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "OFS2", 30000 );
    a_prm->fastadc.CLK_PHASE		= dap_config_get_item_uint32_default(l_cfg, "FASTADC_SETTINGS", "CLK_PHASE", 40 );
    a_prm->fastadc.DAC_SIN		= dap_config_get_item_uint16_default(l_cfg, "FASTADC_SETTINGS", "DAC_SIN", 22000 );


    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        l_str_DAC_offset[strlen(l_str_DAC_offset)-1]=t+49;
        a_prm->fastadc.dac_offsets[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_offset, 32768.0);
    }

    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        l_str_DAC_offset_init_quants[strlen(l_str_DAC_offset_init_quants)-1]=t+49;
        a_prm->fastadc.dac_offsets_init_quants [t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_offset_init_quants, 32768);
    }


    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        l_str_DAC_gain[strlen(l_str_DAC_gain)-1]=t+49;
        a_prm->fastadc.dac_gains[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_gain, 1.0);
    }

    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        l_str_ADC_offset[strlen(l_str_ADC_offset)-1]=t+49;
        a_prm->fastadc.adc_offsets[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_ADC_offset, 0.0);
    }

    for (t=0;t<DRS_DCA_COUNT_ALL;t++){
        l_str_ADC_gain[strlen(l_str_ADC_gain)-1]=t+49;
        a_prm->fastadc.adc_gains[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_ADC_gain, 1.0);
    }

    for (t=0;t<DRS_COUNT;t++){
        l_str_KT[strlen(l_str_KT)-1]=t+49;
        a_prm->fastadc.KT[t] = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_KT, 9200.0);
    }

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_0){
        l_str_ADC_gain[strlen(l_str_DAC_bias)-1]=0 + 49;
        a_prm->fastadc.BIAS[0] = dap_config_get_item_uint16_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_bias , 19000);
    }

    if(g_drs_flags & DRS_INIT_ENABLE_DRS_1){
        l_str_ADC_gain[strlen(l_str_DAC_bias)-1]=1 + 49;
        a_prm->fastadc.BIAS[1] = dap_config_get_item_uint16_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_bias , 19000);
    }


    // Для 9 канала
    l_str_DAC_offset[strlen(l_str_DAC_offset)-1]=9 + 49;
    l_str_DAC_gain[strlen(l_str_DAC_offset)-1]=9 + 49;
    g_ini_ch9.offset  = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_offset, a_prm->fastadc.dac_offsets[0]);
    g_ini_ch9.gain  = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_gain, a_prm->fastadc.dac_gains[0]);


    g_ini_ch9.offset  = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_offset, a_prm->fastadc.dac_offsets[0]);
    g_ini_ch9.gain  = dap_config_get_item_double_default(l_cfg, "FASTADC_SETTINGS", l_str_DAC_gain, a_prm->fastadc.dac_gains[0]);

    dap_config_close( l_cfg );
    return 0;
}

static drs_mode_t s_mode = 0;

/**
 * @brief drs_set_mode
 * @param mode
 */
void drs_set_mode(int a_drs_num, drs_mode_t a_mode)
{
    s_mode = a_mode;
    drs_reg_write(DRS_MODE_REG, a_mode);

    drs_cmd( -1, DRS_CMD_LOAD_N_RUN);
    //
    if (a_mode == DRS_MODE_CAL_TIME)
        drs_set_dac_offset_ch9(DRS_CH9_SHIFT_DEFAULT);
}

/**
 * @brief drs_get_mode
 * @param a_drs_num
 * @return
 */
drs_mode_t drs_get_mode(int a_drs_num)
{
    return s_mode;
}

/**
 * @brief drs_set_dac_input_shift
 * @param addrShift
 * @param value
 */
void drs_set_dac_offsets_quants_all(int a_drs_num,unsigned int a_value)
{
    log_it(L_DEBUG, "Set DAC value 0x%08X", a_value);
    s_dac_shifts_values[a_drs_num] = a_value;
    drs_reg_write(0x8+a_drs_num,a_value);
    //usleep(100);
    drs_start_dac(1);
}

/**
 * @brief drs_dac_shift_input_set_ch9
 * @param a_value
 */
void drs_set_dac_sin(unsigned int a_value)
{
    drs_reg_write(DRS_REG_DAC_SIN ,a_value);
    drs_start_dac();

}

/**
 * @brief drs_dac_shift_input_get
 * @param a_drs_num
 */
unsigned drs_get_dac_offsets_quants_all(int a_drs_num)
{
    return a_drs_num >= 0 ? s_dac_shifts_values[a_drs_num] : 0;
}


/**
 * @brief drs_start_dac
 */
void drs_start_dac()
{
    //unsigned int onAH=1,dacSelect=2;
    drs_reg_write(DRS_REG_START_DAC, 1 );
    usleep(200);
}

/**
 * double *shiftDAC		;
 */

void drs_set_dac_offset_quants(int a_drs_num, unsigned a_ch,  unsigned short a_offset )
{
    unsigned l_value = drs_get_dac_offsets_quants_all(a_drs_num);
    if(a_ch == 0){
        drs_set_dac_offsets_quants_all(a_drs_num, (l_value& 0xFFFF0000) | a_offset );
    }else if (a_ch == 1){
        drs_set_dac_offsets_quants_all(a_drs_num, (l_value& 0xFFFF) |  (a_offset <<16) );
    }else{
        log_it(L_CRITICAL,"Канал для ф-ии установки сдвигов АЦП должен быть 0 или 1, а тут %u ", a_ch);
    }
}

/**
 * @brief drs_set_dac_shift
 * @details Устанавливает смещение ЦАПов
 * @param a_drs_num                       Номер ДРС
 * @param a_values             сдвиги с фронтпанели
 */
void drs_set_dac_offsets_all(int a_drs_num, const double a_values[DRS_CHANNELS_COUNT])
{
    int i;
    assert(a_values);
    assert(a_drs_num>=0);
    float *DAC_gain = g_ini->fastadc.dac_gains + a_drs_num* DRS_CHANNELS_COUNT ;
    float * DAC_offset = g_ini->fastadc.dac_offsets+ a_drs_num* DRS_CHANNELS_COUNT;

    assert(DAC_gain);
    assert(DAC_offset);
    unsigned short l_dac_shifts[DRS_CHANNELS_COUNT] ={0};

    for(i=0;i<DRS_CHANNELS_COUNT;i++) {
        double y= (a_values[i] + 0.5)*16383.0 ;
        y = (y*DAC_gain[i] +DAC_offset[i]);
        l_dac_shifts[i] = y;
        log_it(L_DEBUG, "shiftDAC[%d]=%f",i,a_values[i]);
    }
    drs_set_dac_offsets_quants_all(a_drs_num,((l_dac_shifts[1]<<16)&0xFFFF0000)|l_dac_shifts[0]);
}

/**
 * @brief drs_dac_volts_to_quants
 * @param a_drs_num
 * @param a_ch
 * @param a_value
 * @return
 */
unsigned short drs_dac_volts_to_quants(int a_drs_num, unsigned a_ch, double a_value)
{
    assert(a_drs_num>=0);
    float *DAC_gain = g_ini->fastadc.dac_gains + a_drs_num* DRS_CHANNELS_COUNT ;
    float * DAC_offset = g_ini->fastadc.dac_offsets+ a_drs_num* DRS_CHANNELS_COUNT;

    assert(DAC_gain);
    assert(DAC_offset);
    return floor(fabs((a_value+ 0.5)*16384.0) *DAC_gain[a_ch] +DAC_offset[a_ch]) ;
}

/**
 * @brief drs_dac_shift_set_ch9
 * @param shiftDAC
 */
void drs_set_dac_offset_ch9(double a_shift)
{
    unsigned short l_shift_DAC_value;
    float a_gain = g_ini_ch9.gain;
    float a_offset = g_ini_ch9.offset;
    l_shift_DAC_value= fabs((a_shift+ 0.5)*16384.0) ;
    l_shift_DAC_value=(l_shift_DAC_value*a_gain +a_offset );
    log_it(L_DEBUG, "Set CH9 DAC shift: a_shift_DAC=%f\tl_shift_DAC_value=%d",a_shift,l_shift_DAC_value);
    drs_set_dac_sin( l_shift_DAC_value);
    drs_start_dac();
}

void drs_reg_write(unsigned int reg_adr, unsigned int reg_data)
{
    static pthread_mutex_t l_reg_write_mutex = PTHREAD_MUTEX_INITIALIZER;
    /* get the delay_ctrl peripheral's base address */
    pthread_mutex_lock(&l_reg_write_mutex);
    unsigned int * l_control_mem = (unsigned int *) (control_map + reg_adr*4);
    debug_if(s_debug_more, L_DEBUG, "write: adr=0x%08x (%u), val=0x%08x", reg_adr, reg_adr, reg_data);

    /* write the value */
    *l_control_mem = reg_data;
    usleep(1);
    pthread_mutex_unlock(&l_reg_write_mutex);
}

unsigned int drs_reg_read(unsigned int reg_adr)
{
    unsigned int reg_data;
    control_mem = (unsigned int *) (control_map + reg_adr*4);
    reg_data=(unsigned int)control_mem[0];
//    printf("read: adr=0x%08x, val=0x%08x\n\r", reg_adr, reg_data), fflush(stdout);
    //usleep(100);
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
