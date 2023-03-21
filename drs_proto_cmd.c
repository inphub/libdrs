/*
 * drs_proto_cmd.c
 *
 *  Created on: 25 October 2022
 *      Author: Dmitriy Gerasimov
 */
#include <dap_common.h>

#include "drs.h"
#include "drs_proto.h"
#include "drs_proto_cmd.h"

#include "commands.h"
#include "calibrate.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"
#include "drs_cal_time_global.h"
#include "drs_ops.h"
#include "drs_data.h"

#include "data_operations.h"

#define LOG_TAG "drs_proto_cmd"

size_t g_drs_proto_args_size[DRS_PROTO_CMD_MAX]={
    [CMD_REG_WRITE]           = 2 * sizeof(uint32_t),
    [CMD_REG_READ]            = 1 * sizeof(uint32_t),
    [CMD_READ_MEM]            = 2 * sizeof(uint32_t),

    [CMD_PAGE_READ_DRS1]      = 1 * sizeof(uint32_t),
    [CMD_PAGE_READ_DRS2]      = 1 * sizeof(uint32_t),
    [CMD_INI_FILE_WRITE]      = sizeof(*g_ini),
    [CMD_INI_WRITE]           = sizeof(*g_ini),
    [CMD_READ]                = 4 * sizeof(uint32_t),
    [CMD_READ_X]              = 2 * sizeof(uint32_t), // Read X
    [CMD_READ_Y]              = 3 * sizeof(uint32_t), // Read Y


    [CMD_SHIFT_DAC_SET]       = 1 * sizeof(uint32_t),
    [CMD_FF]                  = 1 * sizeof(uint32_t),
    [CMD_GET_SHIFT]           = 1 * sizeof(uint32_t),
    [CMD_START]               = 3 * sizeof(uint32_t),

    [CMD_CALIBRATE_ABORT ]         = 1 * sizeof(uint32_t),
    [CMD_CALIBRATE_RESULTS ]       = 1 * sizeof(uint32_t),
    [CMD_CALIBRATE_RUN ]           = 7 * sizeof(uint32_t),
    [CMD_CALIBRATE_PROGRESS ]      = 1 * sizeof(uint32_t),
};

#define MAX_PAGE_COUNT 1000
//#define SIZE_FAST MAX_PAGE_COUNT*1024*8*8

static double s_data_y[DRS_CELLS_COUNT] = {0};
static double s_data_x[DRS_CELLS_COUNT_CHANNEL ] = {0};

static int s_read_y_flags_add = DRS_OP_FLAG_ROTATE;

void drs_proto_cmd(dap_events_socket_t * a_es, drs_proto_cmd_t a_cmd, uint32_t* a_cmd_args)
{
    uint32_t l_addr = 0, l_value = 0;
    log_it(L_DEBUG, "---=== Proto cmd 0x%08X ===---", a_cmd);
    switch( a_cmd ) {
        case CMD_IDLE:
            log_it(L_NOTICE, "Client sent idle command, do nothing");
        break;

        case CMD_REG_WRITE: // Write reg size 12
            l_addr=a_cmd_args[0];
            l_value=a_cmd_args[1];
            write_reg(l_addr, l_value);
            log_it(L_DEBUG,"write: adr=0x%08x, val=0x%08x", l_addr, l_value);
            l_value=0;
            dap_events_socket_write_unsafe(a_es, &l_value, sizeof(l_value) );
        break;

        case CMD_REG_READ: //read reg  size 8
            l_addr=a_cmd_args[0];
            l_value=read_reg(l_addr);
            log_it( L_DEBUG,"read: adr=0x%08x, val=0x%08x", l_addr, l_value);
            dap_events_socket_write_unsafe(a_es, &l_value, sizeof(l_value) );
        break;

        case CMD_DATA_READ_DRS1: //read data from drs1 size 16384
            log_it(L_DEBUG, "read data DRS1: [0]=0x%04x,[1]=0x%04x [2]=0x%04x,[3]=0x%04x [4]=0x%04x,[5]=0x%04x [6]=0x%04x,[7]=0x%04x",
                   ((unsigned short *)data_map_drs1)[0], ((unsigned short *)data_map_drs1)[1], ((unsigned short *)data_map_drs1)[2], ((unsigned short *)data_map_drs1)[3], ((unsigned short *)data_map_drs1)[4], ((unsigned short *)data_map_drs1)[5], ((unsigned short *)data_map_drs1)[6], ((unsigned short *)data_map_drs1)[7]);
            drs_proto_out_add_mem( DRS_PROTO(a_es), data_map_drs1, DRS_DATA_MAP_SIZE);
        break;

        case CMD_DATA_READ_DRS2: //read data from drs2 size 16384
            log_it(L_DEBUG, "read data DRS2: [0]=0x%04x,[1]=0x%04x [2]=0x%04x,[3]=0x%04x [4]=0x%04x,[5]=0x%04x [6]=0x%04x,[7]=0x%04x", ((unsigned short *)data_map_drs2)[0], ((unsigned short *)data_map_drs2)[1], ((unsigned short *)data_map_drs2)[2], ((unsigned short *)data_map_drs2)[3], ((unsigned short *)data_map_drs2)[4], ((unsigned short *)data_map_drs2)[5], ((unsigned short *)data_map_drs2)[6], ((unsigned short *)data_map_drs2)[7]);
            drs_proto_out_add_mem( DRS_PROTO(a_es), data_map_drs2, DRS_DATA_MAP_SIZE);
        break;

        case CMD_SHIFT_READ_DRS1:{ //read shift drs1
            uint16_t l_tmpshift=((unsigned long *)data_map_shift_drs1)[0];
            log_it( L_DEBUG, "read shifts: shift DRS1 = 0x%04x, shift DRS1=0x%04x", ((unsigned short *)data_map_shift_drs1)[0], ((unsigned short *)data_map_shift_drs2)[0]);
            dap_events_socket_write_unsafe( a_es, &l_tmpshift, sizeof (l_tmpshift));
            break;
        }

        case CMD_SHIFT_READ_DRS2:{ //read shift drs2
            uint16_t l_tmpshift=((unsigned long *)data_map_shift_drs1)[0];
            log_it( L_DEBUG, "read shifts: shift DRS1 = 0x%04x, shift DRS2=0x%04x", ((unsigned short *)data_map_shift_drs1)[0], ((unsigned short *)data_map_shift_drs2)[0]);
            dap_events_socket_write_unsafe( a_es, &l_tmpshift, sizeof (l_tmpshift));
            break;
        }

        case CMD_INI_READ: //read ini
            assert (g_ini);
            log_it(L_DEBUG, "read ini, sizeof(g_ini)=%zd", sizeof(*g_ini));
            drs_proto_out_add_mem(DRS_PROTO(a_es),g_ini, sizeof(*g_ini));
        break;

        case CMD_READ_MEM: {				// read data from module
            uint32_t adr = a_cmd_args[0]; 	// start address in byte
            uint32_t val = a_cmd_args[1]; 	// number of data in byte
            drs_proto_out_add_mem(DRS_PROTO(a_es), data_map + adr - MEMORY_BASE, val);
            //rbam = read_reg(adr  * 2);
            log_it(L_DEBUG," read_mem: addrs=0x%08X size=%u\n", adr, val);
        } break;


        case CMD_INI_FILE_WRITE: //write file ini
        case CMD_INI_WRITE: //write ini
            log_it(L_DEBUG, "write ini (size %zd)", a_es->buf_in_size);
            memcpy(g_ini, a_cmd_args, sizeof(*g_ini) );
            l_value=0;
            dap_events_socket_write_unsafe ( a_es, &l_value, sizeof(l_value));
        break;

        case CMD_INI_FILE_READ: //read file ini
            log_it(L_DEBUG, "read ini file");
            int filefd;
            filefd = open(g_inipath, O_RDONLY);
            if (filefd == -1) {
                char l_errstr[255];
                int l_errno = errno;
                l_errstr[0] = '\0';
                strerror_r(l_errno, l_errstr, sizeof(l_errstr));
                log_it(L_ERROR, "Can't open \"%s\": %s (code %d)", g_inipath, l_errstr, l_errno);
                break;
            }
            drs_proto_out_add_file(DRS_PROTO(a_es), filefd, true);
        break;

        case CMD_PAGE_READ_DRS1:{//read N page data from DRS1
            log_it(L_DEBUG,"Read %u pages",a_cmd_args[0]);

            size_t t;
            for (t=0; t<a_cmd_args[0]; t++) {
                drs_proto_out_add_mem(DRS_PROTO(a_es),&(((unsigned short *)data_map_drs1)[t*DRS_CELLS_COUNT_ALL]), 0x4000 );
            }
            if (t>0) t--;
            log_it( L_DEBUG, "read page %d data: [0]=0x%04x,[1]=0x%04x [2]=0x%04x,[3]=0x%04x [4]=0x%04x,[5]=0x%04x [6]=0x%04x,[7]=0x%04x\n", t, ((unsigned short *)data_map_drs1)[t*8192+0], ((unsigned short *)data_map_drs1)[t*8192+1], ((unsigned short *)data_map_drs1)[t*8192+2], ((unsigned short *)data_map_drs1)[t*8192+3], ((unsigned short *)data_map_drs1)[t*8192+4], ((unsigned short *)data_map_drs1)[t*8192+5], ((unsigned short *)data_map_drs1)[t*8192+6], ((unsigned short *)data_map_drs1)[t*8192+7]);
            log_it( L_DEBUG, "read shift: %4lu\n", ((unsigned long *)data_map_shift_drs1)[0]);
        }break;

        case CMD_PAGE_READ_DRS2:{//read N page data from DRS2
            log_it(L_DEBUG,"Read %u pages",a_cmd_args[0]);

            size_t t;
            for (t=0; t<a_cmd_args[0]; t++) {
                drs_proto_out_add_mem(DRS_PROTO(a_es),&(((unsigned short *)data_map_drs1)[t*DRS_CELLS_COUNT_ALL]), 0x4000 );
            }
            if (t>0) t--;
            log_it( L_DEBUG, "read page %d data: [0]=0x%04x,[1]=0x%04x [2]=0x%04x,[3]=0x%04x [4]=0x%04x,[5]=0x%04x [6]=0x%04x,[7]=0x%04x\n", t, ((unsigned short *)data_map_drs1)[t*8192+0], ((unsigned short *)data_map_drs1)[t*8192+1], ((unsigned short *)data_map_drs1)[t*8192+2], ((unsigned short *)data_map_drs1)[t*8192+3], ((unsigned short *)data_map_drs1)[t*8192+4], ((unsigned short *)data_map_drs1)[t*8192+5], ((unsigned short *)data_map_drs1)[t*8192+6], ((unsigned short *)data_map_drs1)[t*8192+7]);
            log_it( L_DEBUG, "read shift: %4lu\n", ((unsigned long *)data_map_shift_drs1)[0]);
        }break;


        case CMD_CALIBRATE_RUN:{/**
            calibrate

            a_cmd_args[0] - Ќомер DRS
            a_cmd_args[1] - ключи калибровки, 1 бит амплитудна€,2 локальна€ временна€,3 глобальна€ временна€
            a_cmd_args[2] - N c клиента, количество проходов аплитудной калибровки дл€ каждого уровн€ цапов
            a_cmd_args[3] - Min N с клиента, минимальное число набора статистики дл€ каждой €чейки в локальной калибровке
            a_cmd_args[4] - numCylce, число проходов в глобальной колибровке
            a_cmd_args[5] - количество уровней у амплитудной калибровки count,
            дл€ каждого будет N(из a_cmd_args[2]) проходов,
            при нуле будут выполн€тьс€ два прохода дл€ уровней BegServ и EndServ(о них ниже),
            при не нулевом значении, между  BegServ и EndServ будут включены count дополнительных уровней цапов дл€ амплитудной калибровки
            с a_cmd_args[6] идет массив даблов, первые 2 элемента BegServ и EndServ
            остальные 8 сдвиги цапов

            ¬озврат: 4 байта (int), 0 если всЄ хорошо, код ошибки, если нет
              */

            int l_drs_num = a_cmd_args[0];

            // ѕодготавливаем параметры калибровки
            double * l_levels=(double *)(&a_cmd_args[6]);
            drs_calibrate_params_t l_params = {
                .ampl = {
                    .repeats = a_cmd_args[5],
                    .N = a_cmd_args[2],
                    .splash_gauntlet = DRS_CAL_SPLASH_GAUNTLET_DEFAULT
                },
                .time_global = {
                    .num_cycle = a_cmd_args[4]
                },
                .time_local = {
                    .min_N = a_cmd_args[3],
                    .max_repeats = DRS_CAL_MAX_REPEATS_DEFAULT
                }
            };
            memcpy(l_params.ampl.levels, l_levels,sizeof (l_params.ampl.levels) );

            // ѕодготавливаем флаги
            uint32_t l_flags = 0;
            if ( a_cmd_args[1] & 1)
                l_flags |= DRS_CAL_FLAG_AMPL;
            if ( a_cmd_args[1] & 2)
                l_flags |= DRS_CAL_FLAG_TIME_LOCAL;
            if ( a_cmd_args[1] & 4)
                l_flags |= DRS_CAL_FLAG_TIME_GLOBAL;

            int l_ret = drs_calibrate_run(l_drs_num,l_flags, &l_params);
            dap_events_socket_write_unsafe(a_es, &l_ret, sizeof(l_ret));
        }break;

        case CMD_CALIBRATE_PROGRESS:{
            /*
             * a_cmd_arg[0]  - Ќомер провер€емой DRS
             *
             * ¬озвращает прогресс калибровки от 0 до 100 либо -1 если калибровка не проходила
             */
            int l_ret = drs_calibrate_progress( a_cmd_args[0]);
            dap_events_socket_write_unsafe(a_es, &l_ret, sizeof(l_ret));
        }break;
        case CMD_CALIBRATE_RESULTS :{
            /*
             * a_cmd_arg[0]  - Ќомер провер€емой DRS
             *
             * ¬озвращает коэфициенты, 327'764 байт ( sizeof(coefficients_t) )
             * */
            if(a_cmd_args[0]>= DRS_COUNT ){
                log_it(L_ERROR, "Wrong drs number %u that should be smaller than %u", a_cmd_args[0], DRS_COUNT);
                break;
            }
            drs_proto_out_add_mem(DRS_PROTO(a_es), &(g_drs+a_cmd_args[0])->coeffs, sizeof(g_drs->coeffs));
        } break;
        case CMD_CALIBRATE_ABORT:{
            /*
             * a_cmd_arg[0]  - Ќомер провер€емой DRS
             *
             * ¬озвращает код состо€ни€ от 0 до 1 либо -1 если калибровка уже не идЄт, -2 если неверно задан номер DRS
             */
            int l_ret = drs_calibrate_abort( a_cmd_args[0]);
            dap_events_socket_write_unsafe(a_es, &l_ret, sizeof(l_ret));
        }break;

        case CMD_READ:{
            /*
            read
            a_cmd_args[0]- номер DRS
            a_cmd_args[1]- число страниц дл€ чтени€
            a_cmd_args[2]- флаг дл€ soft start
            a_cmd_args[3]- флаги дл€ передачи массивов Y и X
            */
            int a_drs_num = a_cmd_args[0];
            int l_flags = a_cmd_args[3];
            if(a_drs_num <0 || a_drs_num >=DRS_COUNT){
                log_it(L_ERROR, "Can't get DRS #%d", a_drs_num);
                break;
            }
            drs_t * l_drs = g_drs + a_drs_num;

            l_value=4*(1+((a_cmd_args[3]&24)!=0));
            log_it(L_DEBUG, "Npage %u",a_cmd_args[1]);
            log_it(L_DEBUG, "soft start 0x%08X",a_cmd_args[2]);
            log_it(L_DEBUG, "apply flags 0x%08X",a_cmd_args[3]);
            if((a_cmd_args[1]&1)==1 || true){//soft start

                drs_set_num_pages(l_drs, 1);
                //setSizeSamples(1024);//Peter fix
                drs_data_get_all( l_drs, 0, tmasFast);
            }else{
                drs_read_pages(l_drs, a_cmd_args[1], l_value* 8192, tmasFast, sizeof (tmasFast));
            }

            drs_cal_y_apply(l_drs, tmasFast, s_data_y ,l_flags);

            drs_proto_out_add_mem(DRS_PROTO(a_es), s_data_y,  sizeof(s_data_y) );


        }break;

        case CMD_READ_Y:{
            /*
            read
            a_cmd_args[0]- номер DRS
            a_cmd_args[1]- число страниц дл€ чтени€
            a_cmd_args[2]- флаги
            */
            int a_drs_num = a_cmd_args[0];

            // TODO убрать после исправлений бага в Ui
            // тут мы просто отфильтровываем корректные флаги
            int l_flags =  a_cmd_args[2] & DRS_CAL_APPLY_Y_CELLS;
            if(a_drs_num <0 || a_drs_num >=DRS_COUNT){
                log_it(L_ERROR, "Can't get DRS #%d", a_drs_num);
                break;
            }
            drs_t * l_drs = g_drs + a_drs_num;

            l_value=4*(1+((a_cmd_args[3]&24)!=0));
            log_it(L_INFO, "Read Y cmd: drs_num=%u,npages=%u,flags=0x%08X (%d )",
                   a_cmd_args[0],a_cmd_args[1],a_cmd_args[2],l_flags);

            if((a_cmd_args[1]&1)==1 || true){//soft start
                drs_set_num_pages(l_drs, 1);
                drs_data_get_all( l_drs, s_read_y_flags_add, tmasFast);
            }else{
                drs_read_pages(l_drs, a_cmd_args[1] | s_read_y_flags_add, l_value* 8192, tmasFast, sizeof (tmasFast));
            }

            drs_cal_y_apply(l_drs, tmasFast, s_data_y ,l_flags);


            drs_proto_out_add_mem(DRS_PROTO(a_es), s_data_y,  sizeof(s_data_y) );

        }break;

        case CMD_READ_X:{
            /*
            read
            a_cmd_args[0]- номер DRS
            a_cmd_args[1]- флаги
            */
            int a_drs_num = a_cmd_args[0];
            int l_flags = a_cmd_args[1];

            if(a_drs_num <0 || a_drs_num >=DRS_COUNT){
                log_it(L_ERROR, "Can't get DRS #%d", a_drs_num);
                break;
            }
            drs_t * l_drs = g_drs + a_drs_num;

            for (unsigned n =0; n <DRS_CELLS_COUNT_CHANNEL; n++){
                s_data_x[n] = n;
            }
            drs_cal_x_apply(l_drs, s_data_x ,l_flags);
            drs_proto_out_add_mem(DRS_PROTO(a_es), s_data_x,  sizeof(s_data_x) );

            log_it(L_INFO, "X array requested, apply flags 0x%08X, x[0]=%.4f, x[1]=%.4f, x[2]=%.4f, x[3]=%.4f, x[4]=%.4f ",
                   l_flags, s_data_x[0], s_data_x[1], s_data_x[2], s_data_x[3], s_data_x[4]);
        }break;


        case CMD_SHIFT_DAC_SET:{
            //set shift DAC
            double *shiftDAC=(double *)(&a_cmd_args[0]);
            for(size_t t=0;t<4;t++){
              log_it(L_DEBUG, "%f",shiftDAC[t]);
            }
            for (int d = 0; d < DRS_COUNT; d++)
              drs_dac_shift_set_all(d,&shiftDAC[d * DRS_DAC_COUNT] ,g_ini->fastadc.dac_gains, g_ini->fastadc.dac_offsets);

            drs_dac_set(1);
            l_value=1;
            dap_events_socket_write_unsafe(a_es, &l_value, sizeof(l_value));
        }break;

        case CMD_FF:
            log_it(L_DEBUG,"ff %d",a_cmd_args[0]);
            l_value=15;
            dap_events_socket_write_unsafe(a_es, &l_value, sizeof(l_value));
        break;

        case CMD_GET_SHIFT:{
            //получение массива сдвигов €чеек
            //memcpy(tmasFast,shift,sizeof(unsigned int)*a_cmd_args[0]);
            int a_drs_num = a_cmd_args[0];
            if(a_drs_num <0 || a_drs_num >=DRS_COUNT){
                log_it(L_ERROR, "Can't get DRS #%d", a_drs_num);
                break;
            }
            drs_t * l_drs = g_drs + a_drs_num;
            dap_events_socket_write_unsafe(a_es, &l_drs->shift, sizeof(l_drs->shift));
        }break;

        case CMD_START:{
            int l_drs_num = a_cmd_args[0];
            if(l_drs_num <0 || l_drs_num >=DRS_COUNT){
                log_it(L_ERROR, "Can't get DRS #%d", l_drs_num);
                break;
            }
            drs_t * l_drs = g_drs + l_drs_num;
            //старт
            // a_cmd_args[0] - DRS num
            //a_cmd_args[1]- mode
            //		00 - page mode and read (N page)
            //		01 - soft start and read (one page)
            //		02 - only read
            //		04 - external start and read (one page)
            //a_cmd_args[2] - pages
            switch(a_cmd_args[1]){
                case 0: //00 - page mode and read (N page)
                    drs_set_num_pages(l_drs, a_cmd_args[2]);
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000025, a_cmd_args[2]);
                    setSizeSamples(a_cmd_args[2]*1024);
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000019, a_cmd_args[2]*1024);
                    write_reg(0x00000015, 1<<1);  //page mode
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000015, 1<<1);
                    usleep(100);
                    write_reg(0x00000027, 1);  //external enable
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000027, 1);
                break;
                case 1: //01 - soft start and read (one page)
                    log_it(L_DEBUG, "01 - soft start and read (one page)\n");
                    //drs_set_sinus_signal(true);
                    //drs_set_mode(l_drs_num, MODE_SOFT_START);
                    drs_start(l_drs_num);
                    //write_reg(0x00000015, 0);  //page mode disable

                    /*                                       	 write_reg(0x00000025, 1);   	//pages
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000025, 1);
                    write_reg(0x00000019, 1024);  //size samples
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000019, 1024);
                    //write_reg(0x00000015, 1<<1);  //page mode
                    usleep(100);
                    write_reg(0x00000010, 1);  //soft start
                    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x\n", 0x00000010, 1);
                    */
                break;
                case 2: //02 - only read
                break;

                case 4: //04 - external start and read (one page)
                    printf("04 - external start and read");
                    drs_set_num_pages(l_drs, a_cmd_args[2]);
                    printf("write: adr=0x%08x, val=0x%08x\n", 0x00000025, 1), fflush(stdout);
                    setSizeSamples(1024);
                    printf("write: adr=0x%08x, val=0x%08x\n", 0x00000019, 1024), fflush(stdout);
                    write_reg(0x00000015, 1<<1);  //page mode
                    printf("write: adr=0x%08x, val=0x%08x\n", 0x00000015, 1<<1), fflush(stdout);
                    usleep(100);
                    write_reg(0x00000027, 1);  //external enable
                    printf("write: adr=0x%08x, val=0x%08x\n", 0x00000027, 1), fflush(stdout);
                break;

                default:
                break;
            }
            l_value=0;
            dap_events_socket_write_unsafe(a_es, &l_value, sizeof(l_value));
        }break;
        case CMD_READ_STATUS_N_PAGE:{ //read status and page
            //нужно подправить!!!
            struct fast_compile {
                uint32_t fast_complite;
                uint32_t pages;
            } DAP_ALIGN_PACKED l_reply = {
                .fast_complite = read_reg(0x00000031),//fast complite
                .pages = read_reg(0x0000003D),//num of page complite
            };
            //printf("fast complite: %d, num page complite: %d, slow complite %d\n", a_cmd_args[0], a_cmd_args[1], a_cmd_args[2]),fflush(stdout);
            dap_events_socket_write_unsafe(a_es, &l_reply, sizeof(l_reply));
        }break;
        default: log_it(L_WARNING, "Unknown command %d", a_cmd);
    }
}


