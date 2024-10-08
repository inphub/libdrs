/*
 * drs_proto_cmd.h
 *
 *  Created on: 1 November 2022
 *      Author: Dmitriy Gerasimov <dmitry.gerasimov@demlabs.net>
 */

#include <stdlib.h>
#include <errno.h>

#include <dap_common.h>
#include <dap_string.h>
#include <dap_strfuncs.h>
#include <dap_file_utils.h>
#include <dap_cli_server.h>
#include <dap_tsd.h>

#include "drs.h"
#include "drs_ops.h"
#include "drs_cli.h"
#include "drs_data.h"
#include "drs_dbg.h"
#include "drs_cal.h"
#include "drs_cal_pvt.h"
#include "drs_proto_cmd.h"

#define LOG_TAG "drs_cli"

static int s_callback_start(int a_argc, char ** a_argv, char **a_str_reply);
static int s_callback_read(int a_argc, char ** a_argv, char **a_str_reply);
static int s_callback_calibrate(int argc, char ** argv, char **str_reply);
static int s_callback_help(int argc, char ** argv, char **str_reply);
static int s_callback_exit(int argc, char ** argv, char **str_reply);

static int s_parse_drs_and_check(int a_arg_index, int a_argc, char ** a_argv, char **a_str_reply);
static int s_cli_set(int a_argc, char ** a_argv, char **a_str_reply);
static int s_cli_proto(int a_argc, char ** a_argv, char **a_str_reply);
static int s_cli_sinus(int a_argc, char ** a_argv, char **a_str_reply);
static int s_cli_debug(int a_argc, char ** a_argv, char **a_str_reply);
/**
 * @brief drs_cli_init
 * @return
 */
int drs_cli_init()
{
    // Help
    dap_cli_server_cmd_add ("help", s_callback_help, "Description of command parameters",
                                        "help [<command>]\n"
                                        "\tObtain help for <command> or get the total list of the commands\n"
                                        );
    // Exit
    dap_cli_server_cmd_add ("exit", s_callback_exit, "Stop application and exit",
                "exit\n" );


    // Start DRS
    dap_cli_server_cmd_add ("start", s_callback_start, "Запуск DRS",
                "start -drs <DRS number> -flags <SOFT_START,EXT_START,LOAD_N_RUN,RESET> [-pages <Количество страниц>]\n"
                "\tЗапуск DRS\n"
             );

    // Calibrate
    dap_cli_server_cmd_add ("calibrate", s_callback_calibrate, "Калибровка DRS",
                "\n"
                "calibrate run [-drs <DRS number>]  [-flags <ALL,AMPL,AMPL_CELL,AMPL_INTER,AMPL_9CH,TIME_GLOBAL,TIME_LOCAL>] [-repeats <repeats for Ampl>\n"
                "               -begin <Begin level> -end <End level> -shifts <Shifts for every DCA, splitted with \",\">\n"
                "               -N <Number of levels for ampl cal>] [-num_cycle <Cycles number for time global>]\n "
                "               [-min_N <Minimal N for time local calibration>]\n"
                "\tRun calibration process for specified DRS number or for all channels if number is not specified\n"
                "\t ALL - Включает в себя все остальные флаги. Применяется по умолчанию, если не указан параметр -flags вообще"
                "\t AMPL - enable AMPL_CELL,AMPL_INTER and AMPL_9CH"
                "\t AMPL_CELL - амплитудная калибровка для ячеек"
                "\t AMPL_INTER - межканальная калибровка"
                "\t AMPL_9CH - калибровка 9ого канала"
                "\t TIME_GLOBAL - временная глобальная калибровка"
                "\t TIME_LOCAL - временная локальная калибровка"
                "\n"
                "calibrate state [-drs <DRS number>] [-coeffs <SPLASH,DELTA_TIME,CHAN_K,CHAN_B,K_TIME,K,B,K9,B9>| ALL] [-limit <Array print limits>]\n"
                "\tCheck calibration status for specified DRS number or for all channels if number is not specified\n"
                ""
                "\n"
                "calibrate abort [-drs <DRS number>]\n"
                "\tAbort calibration specified DRS number or for all channels if number is not specified\n"
                "\n"
                           );


    // Set some vars
    dap_cli_server_cmd_add ("set", s_cli_set, "Установить значение для параметра DRS",
                            "\n"
                            "set mode -- <Режим>\n"
                            "\t Установить режим работы DRS\n"
                            "\t Возможные режимы: SOFT_START,EXT_START,PAGE_MODE,CAL_AMPL,CAL_TIME,OFF_INPUTS\n"
                            "\n"
                            "set dac shifts <DAC shifts lists>\n"
                            "\t Установить смещения АЦП в виде списка через запятую\n"
                            "\n"
                            "set freq <Частота>\n"
                            "\t Рабочая частота устройства, допускаются значения: 1,2,3,4 и 5 \n"
                            "\t\t 1 = 1.024 ГГц\n"
                            "\t\t 2 = 2.048 ГГц\n"
                            "\t\t 3 = 3.072 ГГц\n"
                            "\t\t 4 = 4.096 ГГц\n"
                            "\t\t 5 = 4.915200 ГГц\n"
                            "\n"
                            "set flag_end_read -drs <Номер DRS>\n"
                            "\t Установить флаг завершения чтения\n"
                            "\n"
                            "set reg <Номер регистра> <Значение >\n"
                            "\t Установить указанный регистр в указанное значение. Можно писать как в десятичной, так и в шестнадцатиричной форме\n"
                            "\n"
                            "set splash_treshold <значение предела срабатывания алгоритма>\n"
                            "\n"
                            "\t Установить гейн\n"
                            "set gain -drs <номер DRS> -ch <номер канала> -value <значение гейна> [-type <quant|db>]\n"
                            "\n"
                            "\t Установить смещение\n"
                            "set offset -drs <номер DRS> -ch <номер канала> -value <значение гейна> [-type <quant|volt>]\n"
                            "\n"
                            ""
                            );

    // Протокол
    dap_cli_server_cmd_add ("proto", s_cli_proto, "Управление параметрами сетевых протоколов",
                            "current_page <текущая страница>\n"
                            "\t Выбирает текущую страницу для операций чтения\n"
                            );


    // Set off/on sinus generator
    dap_cli_server_cmd_add ("sinus", s_cli_sinus, "Генератор синуса вкл/выкл",
                            "\n"
                            "sinus on|off"
                            "\t Включает/выключает встроенный генератор синуса\n"
                            "\n"
                            ""
                            );

    // Get raw data
    dap_cli_server_cmd_add ("debug", s_cli_debug, "Отладочные команды ",
                            "\n"
                            "debug source file <имя файла>\n"
                            "\t Вместо реальных данных начинает использовать данные из файла <имя файла>\n"
                            "\n"
                            "debug source native\n"
                            "\t Возвращает к работе с данными из памяти или драйвера\n"
                            "\n"
                            );


    // Get raw data
    dap_cli_server_cmd_add ("read", s_callback_read, "Читает данные",
                            "\n"
                            "read write_ready [-drs <DRS num>]\n"
                            "\t Проверка на флаг готовности к записи\n"
                            "\n"
                            "read page [-drs <Номер DRS>] [-limit <Предельное число ячеек для отображения>] [-start_from <номер ячейки>] [-page <Номер страницы>] [-apply <флаги>]\n"
                            "\t Читает данные из указанной ДРС. Для внешнего запуска так же можно указать номер страницы (1 по умолчанию)\n"
                            "\t В случае указания флагов(через запятую) с параметром -apply комманда выводит значения с плавающей точкой. Доступные флаги применения:\n"
                            "\t  CELLS         Амплитудная калибровка\n"
                            "\t  INTERCHANNEL  Межканальная калибровка\n"
                            "\t  SPLASHS       Удаление всплесков\n"
                            "\t  NO_FIX_BAD_CELLS Отменяет удаление \"плохих\" отчётов тем же алгоритмом\n"
                            "\t  NO_ROTATE     Не разворачивать данные\n"
                            "\t  ROTATE_BANK   Разворачивать внутри банка\n"
                            "\t  ROTATE_GLOBAL Разворачивать глобально\n"
                            "\t  PHYS          Приведение к физическим величинам\n"
                            "\t  CH9_ONLY      Только 9ый канал\n"
                            "\t  START_BEFORE  Запускать start и ждать начала и окончание чтения"
                            "\n"
                            "read x -drs <Номер DRS> [-limit <Предельное число ячеек для отображения>] [-start_from <Номер ячейки>] [-apply <флаги>]\n"
                            "\t Генерирует массив Х и применяет к нему калибровки, если указаны. Возможные флаги калибровки:\n"
                            "\t TIME_LOCAL   Применение таймерной локальной калибровки\n"
                            "\t TIME_GLOBAL  Применение таймерной глобальной калибровки\n"
                            "\t ROTATE       Разворот данных\n"
                            "\t PHYS         Приведение к физическим величинам\n"
                            "\n"
                            "read status [-drs <Номер DRS>]\n"
                            "\t Показывает текущий статус чтения, завершено ли и сколько страницов\n"
                            "\n"
                            "read shift [-drs <Номер DRS>]\n"
                            "\t Показывает текущий статус чтения, завершено ли и сколько страницов\n"
                            "\n"
                            "\n"
                            ""
                            );

    return 0;
}

/**
 * @brief drs_cli_deinit
 */
void drs_cli_deinit()
{

}

/**
 * @brief s_parse_drs_and_check
 * @param a_arg_index
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return
 */
static int s_parse_drs_and_check(int a_arg_index, int a_argc, char ** a_argv, char **a_str_reply)
{
    const char * l_arg_drs = NULL;
    dap_cli_server_cmd_find_option_val(a_argv,a_arg_index, a_argc, "-drs", &l_arg_drs);

    if (l_arg_drs){
        int l_drs_num = atoi( l_arg_drs);
        // Проверяем корректность ввода
        if (l_drs_num <0 || l_drs_num >= DRS_COUNT){
            dap_cli_server_cmd_set_reply_text(a_str_reply, "Wrong drs num %u, shoudn't be more than 0 and less than %u",
                                              l_drs_num, DRS_COUNT);
            return -100;
        }
        return l_drs_num;
    }
    return -1;
}

/**
 * @brief s_callback_start
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return
 */
static int s_callback_start(int a_argc, char ** a_argv, char **a_str_reply)
{
    int l_arg_index = 0;

    int l_drs_num = s_parse_drs_and_check(l_arg_index, a_argc, a_argv, a_str_reply);
    if (l_drs_num < 0 ) // Wrong DRS num
        return -1;

    /*double l_shifts[DRS_DAC_COUNT]={30000,30000};
    float l_gains[DRS_DAC_COUNT]= {g_ini->fastadc.dac_gains[l_drs_num*DRS_COUNT],
                                   g_ini->fastadc.dac_gains[l_drs_num*DRS_COUNT + 1]};
    float l_offsets[DRS_DAC_COUNT]={g_ini->fastadc.dac_offsets[l_drs_num*DRS_COUNT],
                                    g_ini->fastadc.dac_offsets[l_drs_num*DRS_COUNT + 1]};

    drs_dac_shift_set_all(l_drs_num, l_shifts,l_gains, l_offsets );*/


    //  Парсим флаги запуска ДРСки
    const char *l_flags_str_c[]={
        [DRS_CMD_SOFT_START ] = "SOFT_START", [ DRS_CMD_EXT_START]  = "EXT_START",
        [DRS_CMD_LOAD_N_RUN]   = "LOAD_N_RUN",   [DRS_CMD_RESET]   = "RESET",
    };
    int l_flags = 0;

    // Читаем аргументы к команде

    // Флаги
    const char * l_flags_str = NULL;
    dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-flags",        &l_flags_str);

    // Количество страниц ( 1 по умолчанию )
    const char * l_pages_str = NULL;
    unsigned l_pages = 1;
    dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-pages",        &l_pages_str);
    if (l_pages_str){
        l_pages = atoi(l_pages_str);
    }

    // Конвертируем флаги
    char ** l_flags_strs = dap_strsplit(l_flags_str, ",",3);
    if ( l_flags_str){
        for (unsigned n = 0; l_flags_strs[n]; n++){
            for(unsigned idx = 0;  (2^idx) < DRS_CMD_MAX ; idx++ ){
                if( dap_strcmp(l_flags_strs[n], l_flags_str_c[2^idx]) == 0 ) {
                    l_flags |= 2^idx;
                }
            }
        }
    }
    dap_strfreev(l_flags_strs);
    //SOFT_START,EXT_START,LOAD_N_RUN,RESET
    drs_set_flag_end_read(l_drs_num, true);
    drs_start(l_drs_num, l_flags, l_pages);
    dap_cli_server_cmd_set_reply_text(a_str_reply,"DRS started #%d with flags 0x%08X (pages %u)"
                                       , l_drs_num, l_flags, l_pages );
    return 0;
}

/**
 * @brief s_callback_data
 * @param argc
 * @param argv
 * @param str_reply
 * @return
 */
static int s_callback_read(int a_argc, char ** a_argv, char **a_str_reply)
{
    enum {
        CMD_NONE =0,
        CMD_WRITE_READY,
        CMD_PAGE,
        CMD_X,
        CMD_STATUS,
        CMD_SHIFT
    };
    int l_arg_index = 1;
    int l_cmd_num = CMD_NONE;

    // Get command
    if( dap_cli_server_cmd_find_option_val(a_argv, l_arg_index, a_argc, "write_ready", NULL)) {
        l_cmd_num = CMD_WRITE_READY;
    }else if( dap_cli_server_cmd_find_option_val(a_argv, l_arg_index, a_argc, "page", NULL)) {
        l_cmd_num = CMD_PAGE;
    }else if( dap_cli_server_cmd_find_option_val(a_argv, l_arg_index, a_argc, "x", NULL)){
        l_cmd_num = CMD_X;
    }else if( dap_cli_server_cmd_find_option_val(a_argv, l_arg_index, a_argc, "status", NULL)) {
        l_cmd_num = CMD_STATUS;
    }else if( dap_cli_server_cmd_find_option_val(a_argv, l_arg_index, a_argc, "shift", NULL)) {
        l_cmd_num = CMD_SHIFT;
    }

    l_arg_index++;

    // Получаем номер DRS
    int l_drs_num = s_parse_drs_and_check(l_arg_index, a_argc, a_argv, a_str_reply);
    if (l_drs_num < -1 ) // Wrong DRS num
        return -1;

    switch (l_cmd_num){
        case CMD_WRITE_READY:{
                bool l_flag_ready=drs_get_flag_write_ready(l_drs_num);
                dap_cli_server_cmd_set_reply_text( a_str_reply, "DRS #%d %s к чтению", l_drs_num, l_flag_ready? "готова": "не готова");
        }break;
      case CMD_SHIFT:{
          const char * l_page_str = NULL;
          dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-page",  &l_page_str);
          unsigned l_page = l_page_str ? atoi(l_page_str) : 0;
          dap_string_t * l_reply = dap_string_new("");
          dap_string_append_printf(l_reply, "DRS read shift for page %u :\n",l_page );

          // Пробегаем по всем DRS, либо только по выбранной
          for(unsigned n = l_drs_num == -1? 0 : l_drs_num; n < DRS_COUNT; n++){
              if( drs_check_flag (n) ){
                  dap_string_append_printf(l_reply, "  #%u: shift=%u shift_bank=%u\n", n, drs_get_shift(n,l_page),drs_get_shift_bank(n,l_page) );
              }
              if (l_drs_num != -1)
                break;
          }
          *a_str_reply = dap_string_free(l_reply, false);
      } break;

        case CMD_STATUS:{
            dap_string_t * l_reply = dap_string_new("DRS read status:\n");
            // Пробегаем по всем DRS, либо только по выбранной
            for(unsigned n = l_drs_num == -1? 0 : l_drs_num; n < DRS_COUNT; n++){
                if( drs_check_flag (n) ){
                    bool l_done  = drs_get_read_ready(n) ; //fast complite
                    uint32_t l_npages = drs_get_pages_ready(n);//num of page complite
                    dap_string_append_printf(l_reply, "  #%u: ready=%s npages=%u", n, l_done ?"true":"false", l_npages);
                }
                if (l_drs_num != -1)
                  break;
            }
            *a_str_reply = dap_string_free(l_reply, false);
        } break;
        case CMD_X:{
            //  -drs <Номер DRS>
            // [-limit <Предельное число ячеек для отображения>]
            // [-start_from <Номер ячейки>]
            // [-apply <флаги>]"

            // Максимальное число ячеек для вывода на экран
              const char * l_limits_str = NULL;
              size_t l_limits = 10;
              dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-limit",  &l_limits_str);
              if (l_limits_str)
                  l_limits = atoi(l_limits_str);

            // Начальный номер ячейки для вывода на экран
            const char * l_start_from_str = NULL;
            size_t l_start_from = 0;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-start_from",  &l_start_from_str);
            if (l_start_from_str)
                l_start_from = atoi(l_start_from_str);

            // Проверяем, не вышли ли за пределы и возвращаемся в них, если вышли
            if(l_start_from + l_limits > DRS_CELLS_COUNT )
                l_limits = DRS_CELLS_COUNT - l_start_from;

            // Применение калибровки
            const char * l_apply_str = NULL;
            int l_apply_flags = 0;
            const size_t c_max_tokens = 100;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-apply",  &l_apply_str);
            if( l_apply_str){
                char ** l_apply_flags_strs = dap_strsplit(l_apply_str, ",", c_max_tokens );
                if (l_apply_flags_strs == NULL){
                    dap_cli_server_cmd_set_reply_text(a_str_reply, "Apply argument is empty");
                    return -4;
                }
                size_t n =0;
                for ( n = 0; l_apply_flags_strs[n] && n <= c_max_tokens ; n ++){
                    char * l_str = l_apply_flags_strs[n];
                    if (dap_strcmp(l_str,"TIME_LOCAL") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_X_TIME_LOCAL ;
                    }else if (dap_strcmp(l_str,"TIME_GLOBAL") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_X_TIME_GLOBAL;
                    }else if (dap_strcmp(l_str,"NO_ROTATE") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_NO_ROTATE;
                    }else if (dap_strcmp(l_str,"PHYS") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_PHYS;
                    }else if (dap_strcmp(l_str,"NO_CUT") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_NO_CUT;
                    }
                }
                dap_strfreev(l_apply_flags_strs);
            }

            // Получаем массив Х
            double * l_x = drs_cal_x_produce(&g_drs[l_drs_num], l_apply_flags);

            // Выводим его
            dap_string_t* l_reply = dap_string_new("");
            dap_string_append_printf(l_reply,"X array for DRS #%d\n", l_drs_num);
            for (size_t t = 0; t < l_limits; t++){
                dap_string_append_printf(l_reply, "%05.05f ", l_x[t + l_start_from]);
                if ( t % 30 == 0 && t > 0)
                    dap_string_append_printf(l_reply, "\n");
            }
            dap_string_append_printf(l_reply, "\n");
            *a_str_reply = dap_string_free(l_reply, false);

            DAP_DELETE(l_x);

        } break;
        case CMD_PAGE:{

            // Максимальное число ячеек для вывода на экран
            const char * l_limits_str = NULL;
            size_t l_limits = 10;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-limit",  &l_limits_str);
            if (l_limits_str)
                l_limits = atoi(l_limits_str);

            // Начальный номер ячейки для вывода на экран
            const char * l_start_from_str = NULL;
            size_t l_start_from = 0;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-start_from",  &l_start_from_str);
            if (l_start_from_str)
                l_start_from = atoi(l_start_from_str);

            // Проверяем, не вышли ли за пределы и возвращаемся в них, если вышли
            if(l_start_from + l_limits > DRS_CELLS_COUNT )
                l_limits = DRS_CELLS_COUNT - l_start_from;

            // Нумер страницы
            const char * l_page_str = NULL;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-page",  &l_page_str);
            int l_page = 0;
            if(l_page_str)
                l_page = atoi(l_page_str);

            // Применение калибровки
            const char * l_apply_str = NULL;
            int l_apply_flags = 0;
            const size_t c_max_tokens = 100;
            bool l_start_before = false;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-apply",  &l_apply_str);
            if( l_apply_str){
                char ** l_apply_flags_strs = dap_strsplit(l_apply_str, ",", c_max_tokens );
                if (l_apply_flags_strs == NULL){
                    dap_cli_server_cmd_set_reply_text(a_str_reply, "Apply argument is empty");
                    return -4;
                }
                size_t n =0;
                for ( n = 0; l_apply_flags_strs[n] && n <= c_max_tokens ; n ++){
                    char * l_str = l_apply_flags_strs[n];
                    if (dap_strcmp(l_str,"CELLS") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_Y_CELLS;
                    }else if (dap_strcmp(l_str,"INTERCHANNEL") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_Y_INTERCHANNEL;
                    }else if (dap_strcmp(l_str,"SPLASHS") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_Y_SPLASHS;
                    }else if (dap_strcmp(l_str, "NO_FIX_BAD_CELLS") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_Y_NO_FIX_BAD_CELLS;
                    }else if (dap_strcmp(l_str,"NO_ROTATE") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_NO_ROTATE;
                    }else if (dap_strcmp(l_str,"ROTATE_BANK") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_ROTATE_BANK;
                    }else if (dap_strcmp(l_str,"ROTATE_GLOBAL") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_ROTATE_GLOBAL;
                    }else if (dap_strcmp(l_str,"PHYS") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_PHYS;
                    }else if (dap_strcmp(l_str,"CH9_ONLY") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_CH9_ONLY;
                    }else if (dap_strcmp(l_str,"START_BEFORE") == 0){
                        l_start_before = true;
                    }else if (dap_strcmp(l_str,"NO_CUT") == 0 ){
                        l_apply_flags |= DRS_CAL_APPLY_NO_CUT;
                    }
                }
                dap_strfreev(l_apply_flags_strs);
            }

            // Флаги чтения данных

            int l_flags_data_read = l_start_before?
                                                drs_get_mode(l_drs_num)== DRS_MODE_EXT_START ||
                                    drs_get_mode(l_drs_num)== DRS_MODE_PAGE_MODE ?
                                        DRS_OP_FLAG_EXT_START : DRS_OP_FLAG_SOFT_START : 0;

            if(l_drs_num!=-1){
                size_t l_buf_size = DRS_CELLS_COUNT * sizeof (unsigned short);
                unsigned short l_buf[DRS_CELLS_COUNT]={0};

                dap_string_t * l_reply = dap_string_new("");
                drs_t * l_drs = &g_drs[l_drs_num];
                int l_ret = drs_data_get_page( l_drs, l_flags_data_read, l_page, l_buf, l_buf_size);
                if (l_ret == 0){
                    dap_string_append_printf(l_reply,"Page read for DRS #%d\n", l_drs_num);
                    if (l_apply_flags){
                        double l_y[DRS_CELLS_COUNT];
                        drs_cal_y_apply(l_drs, l_buf,l_y, l_apply_flags);
                        for (size_t t = 0; t < l_limits; t++){
                            dap_string_append_printf(l_reply, "%05.02f ", l_y[t + l_start_from]);
                            if ( t % 30 == 0 && t > 0)
                                dap_string_append_printf(l_reply, "\n");
                        }
                    }else{
                        for (size_t t = 0; t < l_limits; t++){
                            dap_string_append_printf(l_reply, "0x%04X ", l_buf[t + l_start_from]);
                            if ( t % 30 == 0 && t > 0)
                                dap_string_append_printf(l_reply, "\n");
                        }
                    }
                }else{
                    dap_string_append_printf(l_reply,"ERROR: Page read for DRS #%d returned code %d\n", l_drs_num, l_ret);
                }
                dap_string_append_printf(l_reply, "\n");
                *a_str_reply = dap_string_free(l_reply, false);
            }else{
                dap_string_t * l_reply = dap_string_new("");

                for(size_t n = 0; n < DRS_COUNT ; n++){
                    size_t l_buf_size = DRS_CELLS_COUNT * sizeof (unsigned short);
                    unsigned short l_buf[DRS_CELLS_COUNT]={0};
                    drs_t * l_drs = g_drs + n;
                    int l_ret = drs_data_get_page( l_drs, l_flags_data_read, l_page, l_buf, l_buf_size);
                    if (l_ret == 0){
                        dap_string_append_printf(l_reply,"Page read for DRS %d\n", n);
                        if (l_apply_flags){
                            double l_y[DRS_CELLS_COUNT];
                            drs_cal_y_apply(l_drs, l_buf,l_y, l_apply_flags);
                            for (size_t t = 0; t < l_limits; t++){
                                dap_string_append_printf(l_reply, "%05.02f ", l_y[t + l_start_from]);
                                if ( t % 30 == 0)
                                    dap_string_append_printf(l_reply, "\n");
                            }
                        }else{
                            for (size_t t = 0; t < l_limits; t++){
                                dap_string_append_printf(l_reply, "%04X ", l_buf[t + l_start_from]);
                                if ( t % 30 == 0)
                                    dap_string_append_printf(l_reply, "\n");
                            }
                        }
                    }else{
                        dap_string_append_printf(l_reply,"ERROR: Page read for DRS #%d returned code %d\n", l_drs_num, l_ret);
                    }

                    dap_string_append_printf(l_reply, "\n");
                }
                *a_str_reply = dap_string_free(l_reply, false);
            }
        }break;
        default:
            dap_cli_server_cmd_set_reply_text( a_str_reply, "Wrong call, check help for this command");
            return -3;
    }
    return 0;
}

/**
 * @brief dap_cli_server_cmd_parse_list_doubles
 * @param a_str_reply
 * @param a_str
 * @param a_array
 * @param a_array_size_min
 * @param a_array_size_max
 * @param a_array_size
 * @return
 */
int dap_cli_server_cmd_parse_list_doubles(char ** a_str_reply,  const char * a_str, double * a_array, const size_t a_array_size_min, const size_t a_array_size_max, size_t * a_array_size )
{
    assert(a_array);
    assert(a_array_size_max);
    int l_retcode = 0;
    char ** l_strs = dap_strsplit(a_str, ",",a_array_size_max);
    if (l_strs == NULL){
        dap_cli_server_cmd_set_reply_text(a_str_reply, "List argument is empty");
        l_retcode = -22;
        goto lb_exit;
    }
    size_t n =0;
    for ( n = 0; l_strs[n] && n <= a_array_size_max ; n ++){
        char * l_shift_str = l_strs[n];
        char * l_shift_str_endptr = NULL;
        double l_shift = strtod( l_shift_str, & l_shift_str_endptr);
        if (l_shift_str_endptr == l_shift_str){
            dap_cli_server_cmd_set_reply_text(a_str_reply, "List argument #%i can't be converted to double value (\"%s\"",
                                              n,l_shift_str);
            l_retcode = -23;
            goto lb_exit;
        }
        a_array[n] = l_shift;
        DAP_DELETE(l_shift_str);
    }

    // Сохраняем количество записанных элементов
    if (a_array_size)
        *a_array_size = n;

    // Если мы прошли корректно весь список, то удаляем исходный массив
    if(! l_strs[n] )
        DAP_DEL_Z(l_strs);

    if (n < a_array_size_min){
        dap_cli_server_cmd_set_reply_text(a_str_reply, "List size %u is too small, should be %u at least",
                                          n,a_array_size_min);
        l_retcode = -24;
        goto lb_exit;
    }
    if (n > a_array_size_max){
        dap_cli_server_cmd_set_reply_text(a_str_reply, "List size %u is too big, should be not more than %u",
                                          n,a_array_size_max);
        l_retcode = -25;
        goto lb_exit;
    }
    return 0;
lb_exit:
    if (l_strs){
        dap_strfreev(l_strs);
    }
    return l_retcode;
}

/**
 * @brief s_callback_calibrate
 * @param argc
 * @param argv
 * @param str_reply
 * @return
 */
static int s_callback_calibrate(int a_argc, char ** a_argv, char **a_str_reply)
{
    if(a_argc > 1) {
        int l_arg_index = 1;
        // Читаем доп аргумент (если есть)
        int l_drs_num = s_parse_drs_and_check(l_arg_index,a_argc,a_argv,a_str_reply) ; // -1 значит для всех
        if (l_drs_num < -1)
            return l_drs_num;

        if (strcmp(a_argv[1], "run") == 0 ){ // Subcommand "run"

            // Читаем аргументы к команде
            const char * l_flags_str = NULL;

            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-flags",        &l_flags_str);

            // Конверстируем аргументы и проверяем их корректность

            uint32_t l_flags = 0;
            unsigned l_min_N = DRS_CAL_MIN_N_DEFAULT;
            unsigned l_max_repeats = DRS_CAL_MAX_REPEATS_DEFAULT;
            double l_begin = DRS_CAL_BEGIN_DEFAULT;
            double l_end = DRS_CAL_END_DEFAULT;
            double l_shifts[DRS_DCA_COUNT_ALL] ={};
            unsigned l_repeats = DRS_CAL_REPEATS;
            unsigned l_num_cycle = DRS_CAL_NUM_CYCLE_DEFAULT;
            unsigned l_N = DRS_CAL_N_DEFAULT;
            // Проверяем наличие флагов
            if ( l_flags_str ){
                // Конвертируем флаги
                char ** l_flags_strs = dap_strsplit(l_flags_str, ",",3);
                if (l_flags_strs == NULL){
                    dap_cli_server_cmd_set_reply_text(a_str_reply, "Flags argument is empty");
                    return -21;
                }

                for (size_t i = 0; l_flags_strs[i]; i ++){
                    // Подготавливаем флаги
                    if ( dap_strcmp(l_flags_strs[i], "ALL") == 0)
                                        l_flags |= DRS_CAL_FLAG_AMPL | DRS_CAL_FLAG_AMPL_INTER | DRS_CAL_FLAG_AMPL_9CH
                                            | DRS_CAL_FLAG_TIME_GLOBAL | DRS_CAL_FLAG_TIME_LOCAL;
                    else if ( dap_strcmp(l_flags_strs[i], "AMPL") == 0)
                        l_flags |= DRS_CAL_FLAG_AMPL | DRS_CAL_FLAG_AMPL_INTER | DRS_CAL_FLAG_AMPL_9CH;
                    else if ( dap_strcmp(l_flags_strs[i], "AMPL_CELL") == 0)
                        l_flags |= DRS_CAL_FLAG_AMPL;
                    else if ( dap_strcmp(l_flags_strs[i], "AMPL_INTER") == 0)
                        l_flags |= DRS_CAL_FLAG_AMPL_INTER;
                    else if ( dap_strcmp(l_flags_strs[i], "AMPL_9CH") == 0)
                        l_flags |= DRS_CAL_FLAG_AMPL_9CH;
                    else if ( dap_strcmp(l_flags_strs[i], "TIME_LOCAL") == 0)
                        l_flags |= DRS_CAL_FLAG_TIME_LOCAL;
                    else if ( dap_strcmp(l_flags_strs[i], "TIME_GLOBAL") == 0)
                        l_flags |= DRS_CAL_FLAG_TIME_GLOBAL;
                }
                dap_strfreev(l_flags_strs);
            }else
                l_flags = DRS_CAL_FLAG_AMPL | DRS_CAL_FLAG_AMPL_INTER | DRS_CAL_FLAG_AMPL_9CH
                  | DRS_CAL_FLAG_TIME_GLOBAL | DRS_CAL_FLAG_TIME_LOCAL;


            // Амплитудная калибровка
            if (l_flags & DRS_CAL_FLAG_AMPL){
                int l_ret;
                const char * l_repeats_str = NULL;
                const char * l_N_str = NULL;
                const char * l_begin_str = NULL;
                const char * l_end_str = NULL;
                const char * l_shifts_str = NULL;
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-repeats",      &l_repeats_str);
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-begin",        &l_begin_str);
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-end",          &l_end_str);
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-shifts",       &l_shifts_str);
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-N", &l_N_str);

                // Конвертируем смещения
                if (l_shifts_str)
                    dap_cli_server_cmd_parse_list_double(a_str_reply,l_shifts_str,l_shifts,DRS_CHANNELS_COUNT,DRS_CHANNELS_COUNT,NULL);

                // конвертируем begin
                if (l_begin_str){
                    char * l_begin_str_endptr = NULL;
                    l_begin = strtod( l_begin_str, & l_begin_str_endptr);
                    if (l_begin_str_endptr == l_begin_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "Begin value \"%s\" can't be converted to double", l_begin_str );
                        return -26;
                    }
                }

                // конвертируем end
                if (l_end_str){
                    char * l_end_str_endptr = NULL;
                    l_end = strtod( l_end_str, & l_end_str_endptr);
                    if (l_end_str_endptr == l_end_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "End value \"%s\" can't be converted to double", l_end_str );
                        return -27;
                    }
                }

                // конвертируем repeats
                if(l_repeats_str){
                    char * l_repeats_str_endptr = NULL;
                    l_repeats = strtoul( l_repeats_str, & l_repeats_str_endptr, 10);
                    if (l_repeats_str_endptr == l_repeats_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "Repeats value \"%s\" can't be converted to unsigned integer", l_repeats_str );
                        return -28;
                    }
                }

                // конвертируем N
                if( l_N_str ){
                    char * l_N_str_endptr = NULL;
                    l_N = strtoul( l_N_str, & l_N_str_endptr, 10);
                    if (l_N_str_endptr == l_N_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "levels_count value \"%s\" can't be converted to unsigned integer", l_N_str );
                        return -31;
                    }
                }
            }

            // Локальная временная калибровка
            if (l_flags & DRS_CAL_FLAG_TIME_LOCAL){
                // конвертируем min_N
                const char * l_min_N_str = NULL;
                const char * l_max_repeats_str = NULL;
                char * l_tmp_endptr = NULL;

                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-min_N",        &l_min_N_str);
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-max_repeats",        &l_max_repeats_str);


                if(l_min_N_str){
                    l_min_N = strtoul( l_min_N_str, & l_tmp_endptr, 10);
                    if (l_tmp_endptr == l_min_N_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "MinN value \"%s\" can't be converted to unsigned integer", l_min_N_str );
                        return -29;
                    }
                }
                if (l_max_repeats_str) {
                    l_max_repeats = strtoul( l_max_repeats_str, & l_tmp_endptr, 10);
                    if (l_tmp_endptr == l_max_repeats_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "max repeats value \"%s\" can't be converted to unsigned integer", l_max_repeats_str );
                        return -29;
                    }
                }

            }
            // Глобальная временная калибровка
            if (l_flags & DRS_CAL_FLAG_TIME_GLOBAL){
                const char * l_num_cycle_str = NULL;
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-num_cycle",    &l_num_cycle_str);
                if(l_num_cycle_str){
                    // конвертируем num_cycle
                    char * l_num_cycle_str_endptr = NULL;
                    l_num_cycle = strtoul( l_num_cycle_str, & l_num_cycle_str_endptr, 10);
                    if (l_num_cycle_str_endptr == l_num_cycle_str){
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "num_cycle value \"%s\" can't be converted to unsigned integer", l_num_cycle_str );
                        return -30;
                    }
                }
            }

            // Подготавливаем параметры калибровки
            drs_calibrate_params_t l_params = {
                .ampl = {
                    .repeats = l_repeats,
                    .N = l_N,
                    .splash_treshold = DRS_CAL_SPLASH_TRESHOLD_DEFAULT
                },
                .time_global = {
                    .num_cycle = l_num_cycle
                },
                .time_local = {
                    .min_N = l_min_N,
                    .max_repeats = l_max_repeats
                }
            };

            l_params.ampl.levels[0] = l_begin;
            l_params.ampl.levels[1] = l_end;
            memcpy(l_params.ampl.levels+2, l_shifts,sizeof (l_params.ampl.levels)-2 *sizeof(double) );

            dap_string_t * l_reply = dap_string_new("");

            int l_ret;
            if (l_drs_num == -1) // Если не указан DRS канал, то фигачим все
                for (size_t i = 0; i < DRS_COUNT; i++){
                    l_ret = drs_calibrate_run(i, l_flags, &l_params);
                    if (l_ret == 0)
                        dap_string_append_printf( l_reply, "DRS #u calibration started\n");
                    else
                        dap_string_append_printf( l_reply, "DRS #u calibration start error, code %d\n", l_ret);
                }
            else{ // Если указан, то только конкретный
                l_ret = drs_calibrate_run(l_drs_num, l_flags, &l_params);
                if (l_ret == 0)
                    dap_string_append_printf( l_reply, "DRS #u calibration started\n");
                else
                    dap_string_append_printf( l_reply, "DRS #u calibration start error, code %d\n", l_ret);
            }

            *a_str_reply = dap_string_free(l_reply, false);
        }else if  (strcmp(a_argv[1], "state") == 0 ){ // Subcommand "check"
            dap_string_t * l_reply = dap_string_new("Check drs calibration progress:\n");
            const char * l_coeffs_str = NULL;
            const char * l_limit_str = NULL;
            int l_coeffs_flags = 0;
            unsigned l_limits = 10;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-coeffs",  &l_coeffs_str);
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-limit",  &l_limit_str);

            if (l_limit_str)
              l_limits = atoi(l_limit_str);

            if ( l_coeffs_str == NULL || dap_strcmp( l_coeffs_str, "ALL") == 0){
                log_it(L_DEBUG, "Coeffs: ALL");
                l_coeffs_flags = DRS_COEF_SPLASH |  DRS_COEF_DELTA_TIME | DRS_COEF_CHAN_K | DRS_COEF_CHAN_B |
                                 DRS_COEF_K_TIME | DRS_COEF_K | DRS_COEF_B | DRS_COEF_K9 | DRS_COEF_B9;
            }else{
                log_it(L_DEBUG, "Coeffs: %s",l_coeffs_str);
                char ** l_coeffs_strs = dap_strsplit(l_coeffs_str, ",",10);
                if(l_coeffs_strs){
                    for (size_t i = 0; l_coeffs_strs[i]; i ++){
                        if( dap_strcmp(l_coeffs_strs[i], "SPLASH") == 0 )
                            l_coeffs_flags |= DRS_COEF_SPLASH;
                        else if( dap_strcmp(l_coeffs_strs[i], "DELTA_TIME") == 0 )
                            l_coeffs_flags |= DRS_COEF_DELTA_TIME;
                        else if( dap_strcmp(l_coeffs_strs[i], "CHAN_K") == 0 )
                            l_coeffs_flags |= DRS_COEF_CHAN_K;
                        else if( dap_strcmp(l_coeffs_strs[i], "CHAN_B") == 0 )
                            l_coeffs_flags |= DRS_COEF_CHAN_B;
                        else if( dap_strcmp(l_coeffs_strs[i], "K_TIME") == 0 )
                            l_coeffs_flags |= DRS_COEF_K_TIME;
                        else if( dap_strcmp(l_coeffs_strs[i], "K") == 0 )
                            l_coeffs_flags |= DRS_COEF_K;
                        else if( dap_strcmp(l_coeffs_strs[i], "B") == 0 )
                            l_coeffs_flags |= DRS_COEF_B;
                        else if( dap_strcmp(l_coeffs_strs[i], "K9") == 0 )
                            l_coeffs_flags |= DRS_COEF_K9;
                        else if( dap_strcmp(l_coeffs_strs[i], "B9") == 0 )
                            l_coeffs_flags |= DRS_COEF_B9;
                        else {
                            log_it(L_ERROR, "Unknown flag %s", l_coeffs_strs[i]);
                        }
                        DAP_DELETE(l_coeffs_strs[i]);
                    }
                    DAP_DELETE(l_coeffs_strs);
                }
            }
            if (l_drs_num == -1){ // Если не указан DRS канал, то фигачим все
                for (int i = 0; i < DRS_COUNT; i++){
                    drs_calibrate_state_t *l_cal = drs_calibrate_get_state(i);
                    dap_string_append_printf(l_reply, "--== DRS %d ==--\n", i);
                    drs_cal_state_print(l_reply, l_cal, l_limits, l_coeffs_flags);
                    DAP_DELETE(l_cal);
                }
            }else{ // Если указан, то только конкретный
                drs_calibrate_state_t * l_cal = drs_calibrate_get_state(l_drs_num);
                dap_string_append_printf(l_reply, "--== DRS %d ==--\n", l_drs_num);
                drs_cal_state_print(l_reply, l_cal, l_limits, l_coeffs_flags);
                DAP_DELETE(l_cal);
            }
            *a_str_reply = dap_string_free(l_reply, false);
        } else if ( dap_strcmp( a_argv[1], "abort") == 0){
            dap_string_t * l_reply = dap_string_new("Abort drs calibration:\n");
            if (l_drs_num == -1){ // Если не указан DRS канал, то фигачим все
                for (int i = 0; i < DRS_COUNT; i++){
                    int l_ret = drs_calibrate_abort(i);
                    if (l_ret == 0)
                        dap_string_append_printf(l_reply, "DRS #%d calibration aborted\n", i);
                    else
                        dap_string_append_printf(l_reply, "DRS #%d calibration abort error, code %d\n", i, l_ret);
                }
            }else{ // Если указан, то только конкретный
                int l_ret = drs_calibrate_abort(l_drs_num);
                if (l_ret == 0)
                    dap_string_append_printf(l_reply, "DRS #%d calibration aborted\n", l_drs_num);
                else
                    dap_string_append_printf(l_reply, "DRS #%d calibration abort error, code %d\n", l_drs_num, l_ret);
            }
            *a_str_reply = dap_string_free(l_reply, false);
        } else {
            dap_cli_server_cmd_set_reply_text(a_str_reply, "Unknown subcommand \"%s\"\n", a_argv[1]);
        }
        return 0;
    } else{
        dap_cli_server_cmd_set_reply_text(a_str_reply, "No subcommand. Availble subcommands: run, check. Do \"help calibrate\" for more details\n");
        return -1;
    }
}

/**
 * @brief s_cli_sinus
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return
 */
static int s_cli_sinus(int a_argc, char ** a_argv, char **a_str_reply)
{
  if(a_argc < 2) {
      dap_cli_server_cmd_set_reply_text(a_str_reply, "No required argument off|on " );
      return -2;
  }
  if (dap_strcmp(a_argv[1],"on") == 0 ){ // Включаем синус
      drs_set_sinus_signal(true);
      dap_cli_server_cmd_set_reply_text(a_str_reply,"Sinus switched on\n");
  } else if (dap_strcmp(a_argv[1],"off") == 0 ) { // Выключаем синус
      drs_set_sinus_signal(false);
      dap_cli_server_cmd_set_reply_text(a_str_reply,"Sinus switched off\n");
  }else{
      dap_cli_server_cmd_set_reply_text(a_str_reply,"Wrong argument, can be only \"off\" or \"on\"\n");
      return -3;
  }
  return 0;
}

/**
 * @brief s_cli_proto
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return
 */
static int s_cli_proto(int a_argc, char ** a_argv, char **a_str_reply)
{
    // Описываем субкомманды
    enum {
        CMD_NONE =0,
        CMD_CURRENT_PAGE,
        CMD_Y_ADD_FLAGS,
        CMD_DEBUG_MORE
    };
    const char *l_cmd_str_c[] ={
        [CMD_CURRENT_PAGE] = "current_page",
        [CMD_Y_ADD_FLAGS] = "y_add_flags",
        [CMD_DEBUG_MORE] = "debug_more",
    };

    const int c_cmd_mandatory_args_count[] ={
        [CMD_CURRENT_PAGE] = 1,
        [CMD_Y_ADD_FLAGS] = 1,
        [CMD_DEBUG_MORE] = 0,
    };
    const int c_argc_min = 2;

    if(a_argc < c_argc_min ) {
        dap_cli_server_cmd_set_reply_text(a_str_reply, "No required arguments" );
        return -1;
    }

    const char * l_cmd = a_argv[1]; // Строка с субкоммандой


    // Парсим субкоманды
    int l_cmd_num = CMD_NONE;
    for(int idx = 0; (size_t) idx < sizeof (l_cmd_str_c) / sizeof(typeof (*l_cmd_str_c)); idx ++ ){
        if( dap_strcmp(l_cmd, l_cmd_str_c[idx]) == 0 ) {
            l_cmd_num = idx;
            break;
        }
    }

    if(a_argc <c_argc_min + c_cmd_mandatory_args_count[l_cmd_num] ) {
        dap_cli_server_cmd_set_reply_text(a_str_reply, "Нет обязательных аргументов для команды \"proto %s\"", l_cmd );
        return -2;
    }


    // Сдвигаемся после сабкоманнды до следующего индекса после неё, чтобы распарсить её аргументы
    int l_arg_index = 1;

    // Читаем общие аргументы
    // int l_drs_num = s_parse_drs_and_check(l_arg_index,a_argc,a_argv,a_str_reply) ; // -1 значит для всех

    switch(l_cmd_num){
        case CMD_CURRENT_PAGE:{
            const char * c_page_str = a_argv[c_argc_min];
            unsigned l_page = 0;
            if (dap_sscanf(c_page_str,"%u",&l_page) != 1 ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Не могу распознать \"%s\" как беззнаковое целое");
                return -3;
            }
            g_drs_proto_current_page = l_page;
        }break;
        case CMD_Y_ADD_FLAGS:
        case CMD_DEBUG_MORE:
        default:{
          dap_cli_server_cmd_set_reply_text(a_str_reply, "Не могу распознать команду \"%s\"");
          return -4;
        }
    }

    return 0;
}

/**
 * @brief s_cli_set
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return 0 if all is good, something else if not
 */
static int s_cli_set(int a_argc, char ** a_argv, char **a_str_reply)
{
    // Описываем субкомманды
    enum {
        CMD_NONE =0,
        CMD_MODE,
        CMD_REG,
        CMD_DAC,
        CMD_FLAG_END_READ,
        CMD_DELAY,
        CMD_SPLASH_TRESHOLD,
        CMD_FREQ,
        CMD_GAIN,
        CMD_OFFSET
    };
    const char *l_cmd_str_c[] ={
        [CMD_MODE] = "mode",
        [CMD_REG] = "reg",
        [CMD_DAC] = "dac",
        [CMD_FLAG_END_READ] = "flag_end_read",
        [CMD_DELAY] = "delay",
        [CMD_SPLASH_TRESHOLD] = "splash_treshold",
        [CMD_FREQ] = "freq",
        [CMD_GAIN] = "gain",
        [CMD_OFFSET] = "offset"
    };

    if(a_argc < 3) {
        dap_cli_server_cmd_set_reply_text(a_str_reply, "No required arguments" );
        return -2;
    }

    const char * l_cmd = a_argv[1]; // Строка с субкоммандой

    // Парсим субкоманды
    int l_cmd_num = CMD_NONE;
    for(int idx = 0; (size_t) idx < sizeof (l_cmd_str_c) / sizeof(typeof (*l_cmd_str_c)); idx ++ ){
        if( dap_strcmp(l_cmd, l_cmd_str_c[idx]) == 0 ) {
            l_cmd_num = idx;
            break;
        }
    }

    // Сдвигаемся после сабкоманнды до следующего индекса после неё, чтобы распарсить её аргументы
    int l_arg_index = 1;

    // Читаем общие аргументы
    int l_drs_num = s_parse_drs_and_check(l_arg_index,a_argc,a_argv,a_str_reply) ; // -1 значит для всех

    switch(l_cmd_num){
        case CMD_FREQ:{
            const char * l_freq_str = a_argv[2];
            if ( dap_strncmp(l_freq_str,"1",1)==0 )
                drs_set_freq(DRS_FREQ_1GHz);
            else if ( dap_strncmp(l_freq_str,"2",1)==0 )
                drs_set_freq(DRS_FREQ_2GHz);
            else if ( dap_strncmp(l_freq_str,"3",1)==0 )
                drs_set_freq(DRS_FREQ_3GHz);
            else if ( dap_strncmp(l_freq_str,"4",1)==0 )
                drs_set_freq(DRS_FREQ_4GHz);
            else if ( dap_strncmp(l_freq_str,"5",1)==0 )
                drs_set_freq(DRS_FREQ_5GHz);
        } break;
        case CMD_OFFSET:{
            //"set offset -drs <номер DRS> -ch <номер канала> -value <значение offset>"
            const char * c_arg_ch = NULL, *c_arg_value = NULL, *c_arg_type = NULL;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-ch", &c_arg_ch);
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-value", &c_arg_value);
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-type", &c_arg_type);
            if( ! c_arg_value ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Команда требует указать значение через параметр -value");
                return -2;
            }
            int l_ch = atoi(c_arg_ch);
            if(l_ch < 0 || l_ch > DRS_CHANNELS_COUNT){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Команда требует указать корректный номер канала от 0 до %u, а не эту чушь \"%s\" что вы указали вместо него", DRS_CHANNELS_COUNT, c_arg_ch);
                return -2;
            }

            int l_ret;
            unsigned short l_quants;
            if(c_arg_type && ( dap_strcmp(c_arg_type,"quant") == 0) ){ // Если указаны кванты
                l_quants = atoi(c_arg_value);
            } else { // по умолчанию считаем Вольты
                double l_value = atof(c_arg_value);
                l_quants = drs_dac_volts_to_quants(l_drs_num,l_ch, l_value);
            }
            log_it(L_NOTICE,"Получили значение %u квантов и его и выставляем", l_quants);
            drs_set_dac_offset_quants(l_drs_num, l_ch, l_quants);
            drs_start_dac();

        } break;
        case CMD_GAIN:{
            //"set gain -drs <номер DRS> -ch <номер канала> -value <значение гейна>"
            const char * c_arg_ch = NULL, *c_arg_value = NULL, *c_arg_type = NULL;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-ch", &c_arg_ch);
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-value", &c_arg_value);
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-type", &c_arg_type);
            if( ! c_arg_value ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Команда требует указать значение через параметр -value");
                return -2;
            }
            int l_ch = atoi(c_arg_ch);
            if(l_ch < 0 || l_ch > DRS_CHANNELS_COUNT){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Команда требует указать корректный номер канала от 0 до %u, а не эту чушь \"%s\" что вы указали вместо него", DRS_CHANNELS_COUNT, c_arg_ch);
                return -2;
            }

            int l_ret;
            unsigned l_quants;
            if(c_arg_type && ( dap_strcmp(c_arg_type,"quant") == 0) ){ // Если указаны кванты
                int l_value = atoi(c_arg_value);
                if (l_value < DRS_GAIN_QUANTS_BEGIN || l_value > DRS_GAIN_QUANTS_END ){
                    dap_cli_server_cmd_set_reply_text(a_str_reply, "Значение  %u(в квантах) вышло за диапазон [%u,%u]", l_value,
                                                      DRS_GAIN_QUANTS_BEGIN,DRS_GAIN_QUANTS_END );
                    return -2;
                }
            } else { // по умолчанию считаем Дб
                double l_value = atof(c_arg_value);
                if (l_value < DRS_GAIN_BEGIN || l_value > DRS_GAIN_END ){
                    dap_cli_server_cmd_set_reply_text(a_str_reply, "Значение  %f(в дБ) вышло за диапазон [%f,%f]", l_value,
                                                      DRS_GAIN_BEGIN,DRS_GAIN_END );
                    return -2;
                }
                l_quants = drs_gain_db_to_quants(l_value);
            }
            log_it(L_NOTICE,"Получили значение %u квантов и его и выставляем", l_quants);
            drs_set_gain_quants(l_drs_num, l_ch, l_quants);
            drs_start_dac();
        } break;
        case CMD_MODE:{

            const char * l_arg_mode = NULL;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "-mode", &l_arg_mode);
            if (l_arg_mode == NULL)
                dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "--", &l_arg_mode);

            if( ! l_arg_mode ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Command requires DRS mode with argument -mode <DRS mode>");
                return -2;
            }
            int l_mode = -1;

            //  Парсим режимы ДРСки
            const char *l_mode_str_c[]={
                [DRS_MODE_SOFT_START] = "SOFT_START", [DRS_MODE_EXT_START]  = "EXT_START", [DRS_MODE_PAGE_MODE]  = "PAGE_MODE",
                [DRS_MODE_CAL_AMPL]   = "CAL_AMPL",   [DRS_MODE_CAL_TIME]   = "CAL_TIME",  [DRS_MODE_OFF_INPUTS] = "OFF_INPUTS"
            };
            for(int idx = 0; (size_t) idx < DRS_MODE_MAX ; idx ++ ){
                if( dap_strcmp(l_arg_mode, l_mode_str_c[idx]) == 0 ) {
                    l_mode = idx;
                    break;
                }
            }

            if (l_mode < 0){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Wrong argument mode \"\%s\"",  l_arg_mode);
                break;
            }

            drs_set_mode(-1, l_mode);
            dap_cli_server_cmd_set_reply_text(a_str_reply, "Mode is %s now", l_arg_mode);
        } break;
        case CMD_DELAY:{
            if( l_drs_num == -1 ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Command requires DRS number with argument -drs <DRS number>");
                return -2;
            }

            const char * l_delay_str = NULL;
            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "--",  &l_delay_str);

            if (l_delay_str == NULL){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Command requires -- and value after it");
                return -3;
            }

            unsigned l_delay = atoi (l_delay_str);
            drs_data_set_zap_delay_ns(l_drs_num, l_delay );
            dap_cli_server_cmd_set_reply_text(a_str_reply, "Теперь у DRS #%u стоит задержка в %u наносекунд, ура!", l_drs_num, l_delay );
        }
        case CMD_SPLASH_TRESHOLD:{
            const char * l_treshold_str = a_argv[2];
            unsigned l_treshold = l_treshold_str? atoi(l_treshold_str) : DRS_CAL_SPLASH_TRESHOLD_DEFAULT;
            drs_cal_set_splash_treshold(l_treshold);
            dap_cli_server_cmd_set_reply_text(a_str_reply, "Предел изменения соседних данных в ячейках тепреь равен %u", l_treshold);
        }break;
        case CMD_REG:{
            const char * l_reg_str = a_argv[2];
            const char * l_value_str = a_argv[3];
            char * l_parse_end = NULL;
            size_t l_reg_str_length = dap_strlen(l_reg_str);
            size_t l_value_str_length = dap_strlen(l_value_str);

            unsigned l_reg, l_value, l_reg_base = 10, l_value_base = 10;

            if ( l_reg_str_length > 2 && dap_strncmp(l_reg_str,"0x",2) == 0 ){
                l_reg_base = 16;
                l_reg_str += 2;
            }

            if ( l_value_str_length > 2 && dap_strncmp(l_value_str,"0x",2) == 0 ){
                l_value_base = 16;
                l_value_str += 2;
            }

            l_reg = strtoul(l_reg_str,&l_parse_end, l_reg_base);
            l_value = strtoul(l_value_str,&l_parse_end, l_value_base);

            drs_reg_write(l_reg, l_value);
        }
        break;
        case CMD_FLAG_END_READ:{
            if( l_drs_num == -1 ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Command requires DRS number with argument -drs <DRS number>");
                return -2;
            }
            drs_set_flag_end_read(l_drs_num, true);
        }break;
        case CMD_DAC:{
            if( l_drs_num == -1 ){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "Command requires DRS number with argument -drs <DRS number>");
                return -2;
            }
            drs_t * l_drs = &g_drs[l_drs_num];

            int l_ret;
            const char * l_shifts_str = NULL;
            double l_shifts[DRS_CHANNELS_COUNT];
            bool l_is_ch9 = (drs_get_mode(l_drs->id) == DRS_MODE_CAL_TIME);

            dap_cli_server_cmd_find_option_val(a_argv,l_arg_index, a_argc, "shifts", &l_shifts_str);
            //fill_array(shiftDACValues, &lvl, DRS_CHANNELS_COUNT, sizeof(lvl));
            if(!l_shifts_str){
                dap_cli_server_cmd_set_reply_text(a_str_reply, "No action selected");
                return -1;
            }

            if ( (l_ret = dap_cli_server_cmd_parse_list_double(a_str_reply,l_shifts_str,l_shifts,
                                                               l_is_ch9 ? 1: DRS_CHANNELS_COUNT,DRS_CHANNELS_COUNT,NULL)) < 0 ){
                return l_ret;
            }

            if (l_is_ch9){
                drs_set_dac_offset_ch9(l_shifts[0]);
            } else {
                drs_set_dac_offsets_all(l_drs->id, l_shifts);
            }
        } break;
        default:
            dap_cli_server_cmd_set_reply_text(a_str_reply, "No subcommand \"%s\"", l_cmd);
            return -1;
    }
    return 0;
}

/**
 * @brief s_callback_help
 * @param argc
 * @param argv
 * @param str_reply
 * @return
 */
static int s_callback_help(int argc, char ** argv, char **str_reply)
{
    if(argc > 1) {
        log_it(L_DEBUG, "Help for command %s", argv[1]);
        dap_cli_cmd_t *l_cmd = dap_cli_server_cmd_find(argv[1]);
        if(l_cmd) {
            dap_cli_server_cmd_set_reply_text(str_reply, "%s:\n%s", l_cmd->doc, l_cmd->doc_ex);
            return 0;
        } else {
            dap_cli_server_cmd_set_reply_text(str_reply, "command \"%s\" not recognized", argv[1]);
        }
        return -1;
    } else {
        // TODO Read list of commands & return it
        log_it(L_DEBUG, "General help requested");
        dap_string_t * l_help_list_str = dap_string_new(NULL);
        dap_cli_cmd_t *l_cmd = dap_cli_server_cmd_get_first();
        while(l_cmd) {
            dap_string_append_printf(l_help_list_str, "%s:\t\t\t%s\n",
                    l_cmd->name, l_cmd->doc ? l_cmd->doc : "(undocumented command)");
            l_cmd = (dap_cli_cmd_t*) l_cmd->hh.next;
        }
        dap_cli_server_cmd_set_reply_text(str_reply,
                "Available commands:\n\n%s\n",
                l_help_list_str->len ? l_help_list_str->str : "NO ANY COMMAND WERE DEFINED");
        return 0;
    }
}

/**
 * @brief s_callback_exit
 * @param argc
 * @param argv
 * @param str_reply
 * @return
 */
static int s_callback_exit(int argc, char ** argv, char **str_reply)
{
    UNUSED(argc);
    UNUSED(argv);
    UNUSED(str_reply);
    exit(0);
}


/// Параметры для отладки
static struct {
    const char * name;
    const char * description;
    enum {PARAM_TYPE_STRING = 0x00000001, PARAM_TYPE_INT = 0x00000002, PARAM_TYPE_DOUBLE=0x00000004} type;
    union{
        struct {
            unsigned id;
            unsigned ch;
        } drs;
    } vars;
    enum {DEBUG_PARAM_DRS_SHIFT} kind;
} c_debug_params[] = {
    {.name = "drs_0_shift_0", .description = "Сдвиг для канала 0 у ДРС 0",
     .type =  PARAM_TYPE_INT,
     .vars.drs.id = 0, .vars.drs.ch = 0 },
    {.name = "drs_0_shift_1", .description = "Сдвиг для канала 1 у ДРС 0",
     .type =  PARAM_TYPE_INT,
     .vars.drs.id = 0, .vars.drs.ch = 1 },
    {.name = "drs_1_shift_0", .description = "Сдвиг для канала 0 у ДРС 1",
     .type =  PARAM_TYPE_INT,
     .vars.drs.id = 1, .vars.drs.ch = 0 },
    {.name = "drs_1_shift_1", .description = "Сдвиг для канала 1 у ДРС 1",
     .type =  PARAM_TYPE_INT,
     .vars.drs.id = 1, .vars.drs.ch = 1 }
};

/**
 * @brief s_cli_debug_print_params
 * @param a_reply
 */
static inline void s_cli_debug_print_params(dap_string_t * a_reply)
{
    for( size_t i = 0; i < sizeof(c_debug_params) / sizeof(*c_debug_params) ; i ++ ){
        dap_string_append_printf( a_reply,"%s\t\t%s\n",c_debug_params[i].name, c_debug_params[i].description);
    }
}


/**
 * @brief s_cli_debug_params_check
 * @param a_str_reply
 * @param a_name
 * @return
 */
static inline int s_cli_debug_params_check(char **a_str_reply, const char * a_name)
{
    int c_params_idx = -1;

    for( size_t i = 0; i < sizeof(c_debug_params) / sizeof(*c_debug_params) ; i ++ ){
        if( dap_strcmp(c_debug_params[i].name, a_name) == 0){
            c_params_idx = i;
            break;
        }
    }

    if (c_params_idx == -1 ){
        dap_string_t * l_reply = dap_string_new("");
        dap_string_append_printf( l_reply,"Не могу распознать параметр \"%s\". Варианты: ", a_name );
        s_cli_debug_print_params(l_reply);
        *a_str_reply = dap_string_free(l_reply, false);

    }

    return c_params_idx;
}


/**
 * @brief s_cli_debug
 * @param a_argc
 * @param a_argv
 * @param a_str_reply
 * @return
 */
static int s_cli_debug(int a_argc, char ** a_argv, char **a_str_reply)
{
  // Описываем субкомманды
  enum {
      CMD_NONE =0,
      CMD_SOURCE,
      CMD_GET,
      CMD_SET
  };
  const char *l_cmd_str_c[] ={
      [CMD_SOURCE] = "source",
      [CMD_SET] = "set",
      [CMD_GET] = "get",
  };

  const int c_cmd_mandatory_args_count[] ={
      [CMD_SOURCE] = 1,
      [CMD_SET] = 2,
      [CMD_GET] = 1
  };
  const int c_argc_min = 2;

  if(a_argc < c_argc_min ) {
      dap_cli_server_cmd_set_reply_text(a_str_reply, "Нет обязательных аргументов\n" );
      return -1;
  }

  const char * l_cmd = a_argv[1]; // Строка с субкоммандой


  // Парсим субкоманды
  int l_cmd_num = CMD_NONE;
  for(int idx = 0; (size_t) idx < sizeof (l_cmd_str_c) / sizeof(typeof (*l_cmd_str_c)); idx ++ ){
      if( dap_strcmp(l_cmd, l_cmd_str_c[idx]) == 0 ) {
          l_cmd_num = idx;
          break;
      }
  }

  if(a_argc <c_argc_min + c_cmd_mandatory_args_count[l_cmd_num] ) {
      dap_cli_server_cmd_set_reply_text(a_str_reply, "Нет обязательных аргументов для команды \"dbg %s\"\n", l_cmd );
      return -2;
  }


  // Сдвигаемся после сабкоманнды до следующего индекса после неё, чтобы распарсить её аргументы
  int l_arg_index = 1;

  // Читаем общие аргументы
  // int l_drs_num = s_parse_drs_and_check(l_arg_index,a_argc,a_argv,a_str_reply) ; // -1 значит для всех




  switch(l_cmd_num){
        case CMD_GET:{

            // Читаем имя параметра
            const char * c_name_str = a_argv[c_argc_min];
            int l_idx = s_cli_debug_params_check(a_str_reply, c_name_str);
            if (l_idx < 0 )
                return -3;

            unsigned l_drs_id = c_debug_params[l_idx].vars.drs.id;
            unsigned l_npage = 0;


            switch(c_debug_params[l_idx].kind){
                case DEBUG_PARAM_DRS_SHIFT:{
                    if (g_drs_dbg_source.drs[l_drs_id].shift[l_npage] < 0 )
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "%s[%u] =\n", c_name_str, l_npage );
                    else
                        dap_cli_server_cmd_set_reply_text(a_str_reply, "%s[%u] = %d\n", c_name_str, l_npage,
                                                          g_drs_dbg_source.drs[l_drs_id].shift[l_npage] );
                } break;
            }
      }break;
      case CMD_SET:{

          // Читаем имя параметра
          const char * c_name_str = a_argv[c_argc_min];
          int l_idx = s_cli_debug_params_check(a_str_reply, c_name_str);
          if (l_idx < 0 )
              return -3;

          unsigned l_drs_id = c_debug_params[l_idx].vars.drs.id;
          unsigned l_npage = 0;

          // Читаем значение
          const char * c_value_str = a_argv[c_argc_min+1];
          union {
              double _double;
              int _int;
              const char * _str;
          } l_value;
          int l_value_type = PARAM_TYPE_STRING;

          char * l_endptr = NULL;
          if (c_debug_params[l_idx].type ==  PARAM_TYPE_DOUBLE){
              l_value._double = strtod(c_value_str, &l_endptr);
          }else if (c_debug_params[l_idx].type == PARAM_TYPE_INT){
              if (dap_strncmp(c_value_str,"0x",2) == 0)
                  l_value._int = strtol(c_value_str + 2, &l_endptr,16);
              else
                  l_value._int = strtol(c_value_str, &l_endptr,10);
          }
          int l_errno = errno;
          if (l_endptr <= c_value_str){
              char l_err[32];
              strerror_r(l_errno,l_err,sizeof(l_err));
              dap_cli_server_cmd_set_reply_text(a_str_reply, "Ошибка распознавания у параметра %s значения \"%s\": %s (код %d)",
                                                c_name_str,c_value_str,   l_err, l_errno);
              return -2;
          }

          switch(c_debug_params[l_idx].kind){
              case DEBUG_PARAM_DRS_SHIFT:{
                  g_drs_dbg_source.drs[l_drs_id].shift[0] = l_value._int;
                  dap_cli_server_cmd_set_reply_text(a_str_reply, "%s[l_npage] = %d\n",  c_name_str, l_npage, g_drs_dbg_source.drs[l_drs_id].shift[l_npage] );
              } break;
          }


      } break;
      case CMD_SOURCE:{
          const char * c_subcmd = a_argv[c_argc_min];
          if ( dap_strcmp(c_subcmd, "file") == 0 ){
              if(a_argc <c_argc_min + c_cmd_mandatory_args_count[l_cmd_num] + 1) {
                  dap_cli_server_cmd_set_reply_text(a_str_reply, "Нет обязательного аргумента <имя файла> для команды \"dbg data source <имя файла>\"\n");
                  return -2;
              }

              const char * c_file_path = a_argv[c_argc_min + 1];
              int l_ret;

              if( (l_ret = drs_dbg_source_set_file(c_file_path)) == 0  )
                  dap_cli_server_cmd_set_reply_text(a_str_reply, "Успешно загружено %zd байт отладочных данных в качестве сырых данных по Y\n", g_drs_dbg_source.data.raw_size);
              else
                  dap_cli_server_cmd_set_reply_text(a_str_reply, "Не удалось загрузить файл %u, код ошибки %d \n",c_file_path, l_ret );

              return l_ret;
          }else if (dap_strcmp(c_subcmd, "native") == 0 ){
              int l_ret;

              if( (l_ret = drs_dbg_source_set_native()) == 0)
                  dap_cli_server_cmd_set_reply_text(a_str_reply, "Источник данных переключен на нативный (память или драйвер)\n");
              else
                  dap_cli_server_cmd_set_reply_text(a_str_reply, "Не удалось переключить источник данных на нативный. Код ошибки %d\n", l_ret);

              return l_ret;
          }

      }break;
      default:{
        dap_cli_server_cmd_set_reply_text(a_str_reply, "Не могу распознать команду \"%s\"\n");
        return -4;
      }
  }

  return 0;
}
