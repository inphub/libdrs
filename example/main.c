#include <dap_common.h>
#include <dap_string.h>

#include <drs.h>
#include <drs_cal.h>
#include <drs_data.h>

#define LOG_TAG "main"

static int s_init(void);
static void s_deinit(void);
#ifdef DAP_OS_UNIX
#include <signal.h>
static void sig_exit_handler(int sig_code);
#endif

/// Точка входа
int main(int argc, const char * argv[])
{
    (void) argc; // Отмечаем неиспользуемые аргументы
    (void) argv;

#ifdef DAP_OS_UNIX
    signal(SIGINT, sig_exit_handler); // Выставили обработчик сигналов, чтобы корректно завершаться в юниксах по SIGINT
    signal(SIGHUP, sig_exit_handler);
    signal(SIGTERM, sig_exit_handler);
    signal(SIGQUIT, sig_exit_handler);
    signal(SIGTSTP, sig_exit_handler);
#endif

    int l_ret = s_init(); // Инициируем SDK и библиотеку

    // Если ошиблись с инициализацией, то выходим
    if (l_ret != 0 ){
        return l_ret;
    }


      ///////////////////////////////
     /// Всякое делаем, работаем  //
    ///////////////////////////////

    // Подготавливаем параметры калибровки
    drs_calibrate_params_t l_params = {
        .ampl = {
            .repeats = 1,
            .N = 100,
            .levels = { -0.25, 0.25 }
        },
        .time_global = {
            .num_cycle = 1000
        },
        .time_local = {
            .min_N = 50
        }
    };

    const unsigned l_drs_num = 0; // Будем работать с DRS за номером 0

    // Проверяем, инициализирована ли плата и если нет, то инициализируем
    //if( ! drs_get_inited() ){
    //    drs_cmd_init(NULL);
    //}

    // Запускаем DRS со всеми флагами и заранее подготовленными параметрами калибровки
    if( (l_ret = drs_calibrate_run(l_drs_num, DRS_CAL_FLAG_AMPL | DRS_CAL_FLAG_TIME_LOCAL | DRS_CAL_FLAG_TIME_GLOBAL, &l_params)) < 0 ){
        log_it(L_CRITICAL, "Can't run DRS calibration, code %d", l_ret);
        return -1000;
    }

    // Проверяем статус калибровки и выводим прогрес, если работает
    if ( drs_calibrate_is_running(l_drs_num) )
        log_it(L_INFO, "Progress %u%%", drs_calibrate_progress(l_drs_num));

    // Ждём завершения
    log_it(L_NOTICE, "Waiting for calibration end");
    if ( ( l_ret = drs_calibrate_wait_for_finished(l_drs_num,
                                                2 *60* 1000  /*Максимальное время ожидания, в милисекундах,
                                                               2 минуты в данном случае*/ ) ) < 0 ){
        log_it(L_ERROR, "Waiting for calibration finished returned error code %d", l_ret);
    }

    // Выводим результаты
    drs_calibrate_t *l_cal = drs_calibrate_get_state(l_drs_num);

    log_it(L_NOTICE, " --== Calibration results  ==--");
    dap_string_t * l_str = dap_string_new(NULL);
    drs_cal_state_print(l_str, l_cal, 10 /*Выводить максимум 10 элементов в массиве*/, DRS_COEF_SPLASH |
                       DRS_COEF_DELTA_TIME |DRS_COEF_CHAN_K | DRS_COEF_CHAN_B | DRS_COEF_K_TIME | DRS_COEF_K |
                       DRS_COEF_B | DRS_COEF_K9 | DRS_COEF_B9 );

    log_it(L_INFO, "State: %s", l_str->str);
    dap_string_free(l_str, true);


      ///////////////////////////////
     /// Закончили                //
    ///////////////////////////////

    s_deinit(); // По завершению работ деинициализируем
    return 0;
}

/// Инициализация всего
static int s_init(void)
{
    // Инициализация DAP-SDK
    dap_common_init("example",NULL,NULL);

    dap_log_level_set(L_DEBUG); // Выставляем минимальный уровень логирования. L_DEBUG значит логировать всё

    // Инициализация DRS
    if(drs_init() != 0){
        log_it(L_CRITICAL, "Can't init drs protocol");
        return -12;
    }

    // Инициализация калибровки DRS
    drs_calibrate_init();
    return 0;;
}

/// Деинициализация всего
static void s_deinit(void)
{
    drs_calibrate_deinit();
    drs_deinit();
    dap_common_deinit();
}

#ifdef DAP_OS_UNIX
static void sig_exit_handler(int sig_code) {
    log_it(L_NOTICE, "Got exit code: %d", sig_code);
    s_deinit();
    log_it(L_NOTICE,"Stopped application");
    fflush(stdout);
    exit(0);
}
#endif
