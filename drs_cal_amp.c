#include <assert.h>
#include <unistd.h>
#include <math.h>

#include <dap_common.h>
#include <dap_config.h>

#include "data_operations.h"
#include "commands.h"

#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"
#include "drs_cal.h"
#include "drs_cal_pvt.h"
#include "drs_cal_amp.h"

#define LOG_TAG "drs_cal_amp"

struct amp_context{
    size_t average_count;
    double * average;

    size_t cells_count;
    unsigned short *cells;

    double ***acc; // acc[count][channels][cells]
};

static int s_proc_drs( drs_t * a_drs, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);

static int s_fin_collect(drs_t * a_drs,  drs_cal_args_t * a_args, bool a_ch9_only);
static int s_interchannels_calibration(drs_t * a_drs , drs_cal_args_t * a_args);

static void s_calc_coeff_b( struct amp_context * a_ctx, unsigned a_iteration, unsigned a_repeats, bool a_ch9_only);
static void s_collect_stats_b(drs_t * a_drs,struct amp_context * a_ctx, unsigned a_repeat_iter, bool a_ch9_only);

static void s_get_coefficients(drs_t * a_drs, drs_cal_args_t * a_args,  struct amp_context * a_ctx, bool a_ch9_only);

static bool s_debug_more = false;


int drs_cal_amp_init()
{
    s_debug_more = dap_config_get_item_bool_default(g_config,"drs_cal_amp","debug_more", s_debug_more);
    return 0;
}

void drs_cal_amp_deinit()
{

}

/**
 * @brief drs_cal_amp
 * @param a_drs_num
 * @param a_args
 * @param a_progress
 * @return
 */
int drs_cal_amp(  int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress)
{
    if (a_drs_num == -1){
        int l_ret = -10;
        log_it(L_INFO, "Amplitude calibration for all DRS");
        double l_progress_per_stage_old = a_args->cal->progress_per_stage;
        a_args->cal->progress_per_stage /= (double) DRS_COUNT;
        for (size_t i=0; i < DRS_COUNT; i++ ){
            l_ret = s_proc_drs(g_drs +i, a_args,a_progress);
            if ( l_ret != 0){
                log_it(L_ERROR,"Can't run calibrate amplitude for DRS #%u code %d ",i, l_ret);
                break;
            }
        }
        a_args->cal->progress_per_stage = l_progress_per_stage_old;
        return l_ret;
    }else if (a_drs_num < DRS_COUNT){
        log_it(L_INFO, "Amplitude calibration for DRS #%d", a_drs_num);
        int l_ret = s_proc_drs(g_drs + a_drs_num, a_args,a_progress);
        if ( l_ret != 0){
            log_it(L_ERROR,"Can't run calibrate amplitude for DRS #%u code %d ",a_drs_num, l_ret);
        }
        return l_ret;
    }else{
        log_it(L_WARNING, "Wrong DRS number %u, can't be more than %u", a_drs_num, DRS_COUNT);
        return -11;
    }
}



/**
 * @brief s_proc_drs
 * @param a_drs
 * @param a_args
 * @param a_progress
 * @return
 */
static int s_proc_drs( drs_t * a_drs, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress)
{
    /*	Высчитывает коэффициенты для амплитудной калибровки;
     * coefficients *coef 			структура с кэффициентами;
     * double *calibLvl 			массив [Beg,Mid,End] с фронт панели плюс массив shiftDAC с фронт панели;
     * unsigned int N 				N с фронт панели;
     * parameter_t *prm				ini структура;
     * unsigned int count			количество уровней между
     */
    double * l_shifts = a_args->param.ampl.levels +2;
    double * l_levels = a_args->param.ampl.levels;
    int l_ret = 0;

    unsigned l_dac_shifts_old =  drs_get_dac_shift_quants_all(a_drs->id);
    unsigned short l_dac_old_gains [DRS_COUNT * DRS_CHANNELS_COUNT];

    drs_mode_t l_mode_old = drs_get_mode(a_drs->id);
    log_it(L_INFO, "Calibrate amplitude start: count=%d, begin=%f, end=%f, mode_old=%d, dac_shifts_old=0x%08X",a_args->param.ampl.repeats ,
           l_levels[0], l_levels[1], l_mode_old, l_dac_shifts_old);


    drs_set_sinus_signal(false); // Выключаяем сигнал синусоиды
    drs_set_mode(a_drs->id, DRS_MODE_CAL_AMPL ); // Включаем режим калибровки амплитуды

    drs_get_gain_quants_all(l_dac_old_gains);

    drs_set_gain_quants(a_drs->id,-1, 32);


    // Основная калибровка
    if( s_fin_collect( a_drs, a_args,false) !=0 ) {
        log_it(L_INFO, "No success with fin collect");

        l_ret = -1;
        goto lb_exit;
    }
    log_it(L_NOTICE, "Calibrate fin end: count=%d, begin=%f, end=%f, shifts=%p",a_args->param.ampl.repeats, l_levels[0], l_levels[1], l_shifts);
    //drs_dac_shift_input_set(a_drs->id, l_dac_shifts_old);

    if(a_args->keys.raw & DRS_CAL_FLAG_AMPL_INTER){
        // Межканальная калибровка
        if( s_interchannels_calibration( a_drs, a_args) !=0 ) {
            log_it(L_INFO, "No success with channels calibration");
            l_ret = -2;
            goto lb_exit;
        }
    }
    //drs_dac_shift_input_set(a_drs->id, l_dac_shifts_old);


    if (a_args->keys.raw & DRS_CAL_FLAG_AMPL_9CH ){
        log_it(L_NOTICE, "Calibrate 9 channel");
        // Калибруем 9ый канал
        drs_set_mode(a_drs->id, DRS_MODE_CAL_TIME);

        // Собираем данные для 9 канала
        if( s_fin_collect( a_drs, a_args, true) !=0 ) {
            log_it(L_INFO, "No success with fin collect");
            l_ret = -1;
            goto lb_exit;
        }
    }
    // Выключаем режим калибровки амплитуды

    drs_set_mode(a_drs->id, l_mode_old );
    drs_set_dac_shift_quants_all(a_drs->id, l_dac_shifts_old);
    drs_set_gain_quants_all(l_dac_old_gains);

    l_dac_shifts_old = drs_get_dac_shift_quants_all(a_drs->id);

    log_it(L_NOTICE, "Channels calibrate ends: count=%d, begin=%f, end=%f, dac_shifts=%08X", a_args->param.ampl.repeats
           , l_levels[0], l_levels[1], l_dac_shifts_old);

    a_drs->coeffs.indicator|=1;
    return l_ret;
lb_exit:
    drs_set_mode(a_drs->id, l_mode_old ); // Выключаем режим калибровки амплитуды
    drs_set_dac_shift_quants_all( a_drs->id, l_dac_shifts_old);
    return l_ret;


}

/**
 * @brief s_fin_collect
 * @param a_drs
 * @param a_args
 * @return
 */
static int s_fin_collect( drs_t * a_drs, drs_cal_args_t * a_args, bool a_ch9_only)
{
    /**
     * Собирает данные по котором будет калибровать
     * double *calibLvl 			массив [Beg,Mid,End] с фронт панели;
     * unsigned int N				N с фронт панели;
     * float *DAC_gain				массив DAC_gain из ini;
     * float *DAC_offset			массив DAC_offset из ini;
     * coefficients *coef			структура с кэффициентами;
     */
    assert(a_drs);
    assert(a_args);
    int l_ret = 0;
    double dh=0.0,lvl=0.0;
    double * calibLvl = a_args->param.ampl.levels;
    unsigned l_count = a_args->param.ampl.repeats +2;
    unsigned l_cells_count = a_ch9_only ?DRS_CELLS_COUNT_BANK*DRS_CHANNELS_COUNT  : DRS_CELLS_COUNT  ;
    unsigned l_cells_count_ch = a_ch9_only ?DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL  ;

    struct amp_context l_ctx = {
      .average_count =       DRS_CHANNELS_COUNT * l_count,
      .average =          DAP_NEW_SIZE(   double,  DRS_CHANNELS_COUNT * l_count * sizeof(*l_ctx.average)),

      .cells_count     =     l_cells_count,
      .cells =     DAP_NEW_Z_SIZE( unsigned short, l_cells_count * sizeof (unsigned short) ),

      .acc       =   DAP_NEW_Z_SIZE( double**,l_count* sizeof (*l_ctx.acc)   ),

    };
    assert(l_ctx.acc);
    assert(l_ctx.average);
    assert(l_ctx.cells);

    // Создаём трёхмерный массив аккумулятор [<количество итераций>][<число каналов>][<число ячеек на каждый канал>]
    for(unsigned iter=0; iter < l_count; iter++){
        l_ctx.acc[iter] = DAP_NEW_Z_SIZE( double *, DRS_CHANNELS_COUNT* sizeof(*l_ctx.acc[iter]) );
        for(unsigned ch=0; ch < DRS_CHANNELS_COUNT; ch++){
            l_ctx.acc[iter][ch] = DAP_NEW_Z_SIZE(double, DRS_CELLS_COUNT_CHANNEL * sizeof (*l_ctx.acc[iter][ch]));
        }
    }

    // Зануляем коэфициенты
    if (a_ch9_only){
        memset(&a_drs->coeffs.b9,0 , sizeof (a_drs->coeffs.b9));
        memset(&a_drs->coeffs.k9,0 , sizeof (a_drs->coeffs.k9));
    }else{
        for (unsigned ch = 0; ch < DRS_CHANNELS_COUNT; ch++){
            memset(&a_drs->coeffs.b[ch],0 , sizeof (a_drs->coeffs.b[ch]));
            memset(&a_drs->coeffs.k[ch],0 , sizeof (a_drs->coeffs.k[ch]));
        }
    }


    dh=(calibLvl[1]-calibLvl[0])/((double) (l_count-1));

    log_it(L_NOTICE,"--Collecting coeficients in %u iterations, step length %f", l_count, dh);
    unsigned l_repeats = 0;

    double l_progress_substages = 1.0;
    if(a_args->keys.raw & DRS_CAL_FLAG_AMPL_INTER)
      l_progress_substages += 1.0;
    if(a_args->keys.raw & DRS_CAL_FLAG_AMPL_9CH)
      l_progress_substages += 1.0;

    double l_progress_total = ((double) a_args->cal->progress_per_stage) /l_progress_substages;
    double l_progress_step =  l_progress_total / (double) (l_count * a_args->param.ampl.N);
    double l_progress = a_args->cal->progress;
    double l_progress_old = l_progress;

    for(unsigned i = 0; i < l_count; i++){
        debug_if(s_debug_more, L_INFO, "Repeat #%u", i);
        lvl = calibLvl[0]+dh*((double)i);
        if(a_ch9_only){
            drs_set_dac_shift_ch9(lvl);
        }else{
            double shiftDACValues[DRS_CHANNELS_COUNT ];
            fill_array(shiftDACValues, &lvl, DRS_CHANNELS_COUNT, sizeof(lvl));
            drs_set_dac_shift(a_drs->id, shiftDACValues);
        }

        for(unsigned k=0;k< a_args->param.ampl.N;k++){
            if(drs_data_get_page_from_first(a_drs,DRS_OP_FLAG_CALIBRATE| DRS_OP_FLAG_SOFT_START, l_ctx.cells, l_cells_count * sizeof (unsigned short) )!=0){
                log_it(L_ERROR, "data not read on %u::%u iteration", i,k);
                l_ret = -1;
                goto lb_exit;
            }
            if(a_drs->shift_bank>1023){
               log_it(L_ERROR, "shift index went beyond on %u::%u iteration", i,k);
               l_ret = -2;
               goto lb_exit;
            }
            s_collect_stats_b(a_drs,&l_ctx,i, a_ch9_only);
            l_repeats++;
            l_progress += l_progress_step;
            a_args->cal->progress = floor(l_progress);
        }

        // Немного дополнительного отладочного вывода
        if( s_debug_more){
            for(unsigned n = 0; n < 10; n++){
                log_it(L_INFO, "cells[%u]=%u", n,  l_ctx.cells[n]);
            }
            for(unsigned c = 0; c < DRS_CHANNELS_COUNT; c++){
                if (a_ch9_only)
                    c = DRS_CHANNEL_9;
                for(unsigned n = 0; n < 10; n++){
                    log_it(L_INFO, "acc[%u][%u][%u]=%f", i,c,n, l_ctx.acc[i][c][n]);
                }
                if (a_ch9_only)
                  break;
            }
        }
        s_calc_coeff_b(&l_ctx, i, a_args->param.ampl.N, a_ch9_only );
        //debug_if(s_debug_more, L_DEBUG, "acc[]={%f,%f,%f,%f...}", l_ctx.acc[0], l_ctx.acc[1], l_ctx.acc[2], l_ctx.acc[3]);
    }
    //debug_if(s_debug_more, L_DEBUG, "acc[]={%f,%f,%f,%f...}", l_ctx.acc[0], l_ctx.acc[1], l_ctx.acc[2], l_ctx.acc[3]);

    if(a_ch9_only){
        for(unsigned i = 0; i < l_count; i++){
            unsigned idx = i*DRS_CHANNELS_COUNT + DRS_CHANNEL_9;
            debug_if(s_debug_more, L_DEBUG, "average[%u]=%f", idx, l_ctx.average[idx]);
        }

    }else {
        for(unsigned i = 0; i < l_count * DRS_CHANNELS_COUNT; i++){
            debug_if(s_debug_more, L_DEBUG, "average[%u]=%f", i, l_ctx.average[i]);
        }
    }
    debug_if(s_debug_more, L_INFO, "Collected stats in %u repeats", l_repeats);

    s_get_coefficients(a_drs, a_args, &l_ctx, a_ch9_only);

    log_it(L_NOTICE,"Collected coeficients in %u repeats",l_repeats);

    // Восстанавливаем уровни
//    double shiftDACValues[DRS_CHANNELS_COUNT ];
//    lvl = (calibLvl[1]-calibLvl[0])/2.0 + calibLvl[0];
//    fill_array(shiftDACValues, &lvl, DRS_DAC_COUNT, sizeof(lvl));
//    drs_dac_shift_set_all(a_drs->id, shiftDACValues,g_ini->fastadc.dac_gains, g_ini->fastadc.dac_offsets);

lb_exit:
    a_args->cal->progress =  l_progress_old + l_progress_total;

    // Очищаем буфер с результатами постраничного чтения
    if(l_ctx.cells)
        DAP_DELETE(l_ctx.cells);

    // Очищаем массив аккумулятора
    if(l_ctx.acc){
        for(unsigned iter=0; iter < l_count; iter++){
            if (l_ctx.acc[iter]){
                for(unsigned ch=0; ch < DRS_CHANNELS_COUNT; ch++){
                    DAP_DELETE(l_ctx.acc[iter][ch]);
                }
                DAP_DELETE(l_ctx.acc[iter]);
            }
        }

        DAP_DELETE(l_ctx.acc);
    }

    // Очищаем массив средних
    if(l_ctx.average)
        DAP_DELETE(l_ctx.average);

    return l_ret;
}

/**
 * @brief s_channels_calibration
 * @param a_drs
 * @param a_args
 * @return
 */
static int s_interchannels_calibration(drs_t * a_drs , drs_cal_args_t * a_args)
{
    unsigned l_count = a_args->param.ampl.repeats + DRS_CAL_MIN_REPEATS_DEFAULT;
    double dh,shiftDACValues[DRS_CHANNELS_COUNT],
        l_lvl,average[DRS_CHANNELS_COUNT*l_count],
        l_d_buf[DRS_CELLS_COUNT],
        x[l_count],
        y[l_count];
    unsigned short l_cells[DRS_CELLS_COUNT];
    double *calibLvl = a_args->param.ampl.levels;
    unsigned int t=0,ch;
    dh=(calibLvl[1]-calibLvl[0])/((double) (l_count-1) );
    if (l_count == 0){
      log_it(L_ERROR, "Repeats %d is wrong, its sum with %d should be not less than 0", a_args->param.ampl.repeats,
             DRS_CAL_MIN_REPEATS_DEFAULT );
      return -10;
    }
    log_it(L_NOTICE,"--Межканальная калибровка--");

    double l_progress_substages = 1.0;
    if(a_args->keys.raw & DRS_CAL_FLAG_AMPL_INTER)
      l_progress_substages += 1.0;
    if(a_args->keys.raw & DRS_CAL_FLAG_AMPL_9CH)
      l_progress_substages += 1.0;

    double l_progress_total = ((double) a_args->cal->progress_per_stage) /l_progress_substages;
    double l_progress_step  = l_progress_total / (double) ( l_count );
    double l_progress       = a_args->cal->progress;


    for(t=0;t<l_count;t++){
        l_lvl=calibLvl[0]+dh*((double)t);
        fill_array(shiftDACValues,&l_lvl,DRS_DAC_COUNT,sizeof(l_lvl));
        drs_set_dac_shift(a_drs->id, shiftDACValues);
        if (drs_data_get_page_first(a_drs,DRS_OP_FLAG_SOFT_START , l_cells ) != 0){
            log_it(L_ERROR,"data not read on iteration %u", t);
            return -1;
        }
        if(a_drs->shift_bank>1023){
            log_it(L_ERROR,"shift index went beyond on iteration %u", t);
            return -2;
        }
        drs_cal_y_apply(a_drs, l_cells,l_d_buf, DRS_CAL_APPLY_Y_CELLS | DRS_CAL_APPLY_Y_SPLASHS );
        //s_find_splash( a_drs, l_d_buf, a_args->param.ampl.splash_treshold,false);
        getAverage(&average[t*DRS_CHANNELS_COUNT],l_d_buf,DRS_CELLS_COUNT_CHANNEL,DRS_CHANNELS_COUNT);

        l_progress += l_progress_step;
        a_args->cal->progress = floor(l_progress);
    }

    for(ch=0;ch<DRS_CHANNELS_COUNT;ch++){
        for(t=0;t<l_count;t++){
            x[t] = (DRS_ADC_VOLTAGE_BASE + calibLvl[0]+dh*((double) t) )*DRS_ADC_TOP_LEVEL;
            y[t] = average[DRS_CHANNELS_COUNT*t+ch] - x[t];
        }
        getCoefLine(y,x, l_count, &a_drs->coeffs.chanB[ch],
                    &a_drs->coeffs.chanK[ch]);
    }

    // Restore levels
    l_lvl = 0.0;
    fill_array(shiftDACValues,&l_lvl,DRS_DAC_COUNT,sizeof(l_lvl));
    drs_set_dac_shift(a_drs->id, shiftDACValues);

    return 0;
}


/**
 * @brief s_get_coefficients   Вычисляет коэфициенты
 * @param a_drs                Объект DRS
 * @param a_args               Аргументы калибровки
 * @param a_acc                Сумма знчаений для ячейки
 * @param a_average            Средние значения по каналам
 */
static void s_get_coefficients(drs_t * a_drs, drs_cal_args_t * a_args, struct amp_context * a_ctx, bool a_ch9_only)
{
    unsigned int l_cell_id,l_ch_id;
    unsigned int l_count = a_args->param.ampl.repeats+2;
    unsigned l_cells_proc_count =  a_ch9_only ?DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL ;
    double * yArr = l_count ? DAP_NEW_STACK_SIZE(double, sizeof(double) * l_count) : NULL;
    double * xArr = l_count ? DAP_NEW_STACK_SIZE(double, sizeof(double) * l_count): NULL;
    double * l_levels = a_args->param.ampl.levels;
    double dh;
    dh=(l_levels[1] - l_levels[0])/(l_count - 1);
    for(l_ch_id=0; l_ch_id < DRS_CHANNELS_COUNT; l_ch_id++) {
        if(a_ch9_only)
            l_ch_id = DRS_CHANNEL_9;

        for(l_cell_id=0; l_cell_id < l_cells_proc_count;l_cell_id ++){
            for(unsigned l_iter=0; l_iter < l_count; l_iter++){
                unsigned l_average_id = l_iter*DRS_CHANNELS_COUNT+l_ch_id;

                xArr[l_iter]= (0.5 + l_levels[0] +  dh * ( (double)l_iter)  ) * 16384.0;

                yArr[l_iter]=a_ctx->acc[l_iter][l_ch_id][l_cell_id] - a_ctx->average[l_average_id];
                //log_it(L_INFO, "l_cell_id=%u|| acc[] = %f,  xArr[%u]=%f, yArr[%u]=%f", l_cell_id,
                //       a_ctx->acc[l_iter][l_ch_id][l_cell_id],
                //       l_iter, xArr[l_iter],  l_iter, yArr[l_iter] );
            }
            if (a_ch9_only)
                getCoefLine(yArr, xArr, l_count, &a_drs->coeffs.b9[l_cell_id], &a_drs->coeffs.k9[l_cell_id]);
            else
                getCoefLine(yArr, xArr, l_count, &a_drs->coeffs.b[l_ch_id][l_cell_id], &a_drs->coeffs.k[l_ch_id][l_cell_id]);
        }

        if(a_ch9_only) // Если только 9ый канал, то тут выходим
            break;
    }
}

/**
 * @brief s_collect_stats_cells         Собирает статистику для каждой ячкейки
 * @param a_drs                         Объект DRS
 * @param a_acc                         сумма знчаений для ячейки
 * @param a_buff                        массив данных
 */
static void s_collect_stats_b(drs_t * a_drs,struct amp_context * a_ctx, unsigned a_repeat_iter, bool a_ch9_only)
{
    //debug_if(s_debug_more, L_DEBUG, "Collect stats ( shift %u )  ", a_drs->shift);
    double **l_acc = a_ctx->acc[a_repeat_iter];
    unsigned short *l_values = a_ctx->cells;

    unsigned int l_rotate_index;
    unsigned l_cells_proc_count =  a_ch9_only ?DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL ;
    for(unsigned l_ch_id = 0; l_ch_id< DRS_CHANNELS_COUNT; l_ch_id++){
        if(a_ch9_only)
            l_ch_id = DRS_CHANNEL_9;
        for(unsigned l_cell_id=0; l_cell_id < l_cells_proc_count ;l_cell_id++){

            l_rotate_index = a_ch9_only? (a_drs->shift_bank + l_cell_id) & 1023 :
                                       (a_drs->shift_bank + (l_cell_id&1023)) & 1023 ;
            assert( a_drs->shift_bank < l_cells_proc_count);

            unsigned l_acc_index =  a_ch9_only?  l_rotate_index :
                                                (l_cell_id&3072) | l_rotate_index;
            unsigned l_cell_id_shift = l_cell_id * DRS_CHANNELS_COUNT + l_ch_id ;

            l_acc[l_ch_id][l_acc_index] += l_values[ l_cell_id_shift];
            //if(!l_acc_index && s_debug_more)
            //    log_it(L_NOTICE, "l_acc[%u][%u][0] += l_values[%u]{%u}", a_repeat_iter, l_ch_id, l_cell_id_shift, l_values[l_cell_id_shift]);
            //if( l_values[l_cell_id_shift + l_cell_id] > 10000.0)
            //if (  l_values[l_cell_id_shift + l_cell_id] < 16000)
            //    debug_if(s_debug_more, L_DEBUG, "l_acc_index = %u, l_cell_id = %u, l_cell_id_shift = %u, l_values = [%u]", l_acc_index, l_cell_id,
             //            l_cell_id_shift, l_values[l_cell_id_shift + l_cell_id]);
        }
        if(a_ch9_only) // Если только 9ый канал, то тут выходим
            break;
    }

}

/**
 * @brief s_calc_coeff_b  Вычисляет коэффиценты для амплитудной калибровки ячеек
 * @param a_acc
 * @param a_average
 * @param a_repeats
 */
static void s_calc_coeff_b( struct amp_context * a_ctx, unsigned a_iteration, unsigned a_repeats, bool a_ch9_only)
{
    debug_if(s_debug_more, L_DEBUG, "Calc coef b");

    double l_value=0;
    double * l_average = &a_ctx->average[a_iteration *DRS_CHANNELS_COUNT];
    double ** l_acc = a_ctx->acc[a_iteration];

    unsigned l_cells_proc_count =  a_ch9_only ? DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL ;


    for(unsigned l_ch_id=0; l_ch_id<DRS_CHANNELS_COUNT;l_ch_id++) { // пробег по каналам
        if(a_ch9_only) // Если только 9ый канал, то работает только по одному
            l_ch_id = DRS_CHANNEL_9;

        l_average[l_ch_id]=0; // зануляемся
        for(unsigned l_cell_id=0; l_cell_id<l_cells_proc_count; l_cell_id++){ // пробег по ячейкам канала
            l_value = l_acc[l_ch_id][l_cell_id]/ ((double)a_repeats);
            l_average[l_ch_id] += l_value;
            l_acc[l_ch_id][l_cell_id ] = l_value;
        }
        if( l_average[l_ch_id] == 0.0)
            log_it(L_WARNING, "Average equals 0 in B coeff calculation ");
        else
          l_average[l_ch_id] = l_average[l_ch_id]/ ((double)l_cells_proc_count);

        if(a_ch9_only) // Если только 9ый канал, то тут выходим
            break;
    }
}


/**
 * @brief drs_cal_amp_remove_splash
 * @param a_drs
 * @param a_Y
 * @param a_treshold
 * @param a_flags
 */
void drs_cal_amp_remove_splash(drs_t * a_drs, double*a_Y, double a_treshold, int a_flags)
{

    const unsigned l_cells_proc_count = drs_cal_get_y_count_after_cuts()/DRS_CHANNELS_COUNT -3;

    bool l_found_smth [DRS_CHANNELS_COUNT][DRS_CELLS_COUNT_CHANNEL ] = {};
    const unsigned c_bad_cells[]={DRS_CAL_AMP_BAD_CELLS};
    unsigned l_bad_cells_shifted[]={DRS_CAL_AMP_BAD_CELLS};

    if(! (a_flags& DRS_CAL_APPLY_Y_NO_FIX_BAD_CELLS) ){
        log_it(L_INFO,"Исправляем плохие ячейки ( shift = %u, shift_bank = %u)", a_drs->shift, a_drs->shift_bank);
        for(unsigned ch=0;ch< DRS_CHANNELS_COUNT; ch++){
            for (unsigned i = 0; i < sizeof (l_bad_cells_shifted)/ sizeof (unsigned); i++){
                unsigned b = c_bad_cells[i] & DRS_BANK_OUT_MASK;
                unsigned p = (c_bad_cells[i] & DRS_BANK_MASK  - a_drs->shift_bank ) & DRS_BANK_MASK;

                l_bad_cells_shifted[i] =  ( (b + p /*- a_drs->shift*/ ) & DRS_GLOBAL_MASK ) /*-2*/;

                //l_rotate_index = a_ch9_only? (a_drs->shift_bank + l_cell_id) & 1023 :
                //                           (a_drs->shift_bank + (l_cell_id&1023)) & 1023 ;
                log_it(L_INFO, "плохая ячейка %u/%u", l_bad_cells_shifted[i], c_bad_cells[i] );
                l_found_smth[ch][l_bad_cells_shifted[i]] = true;
            }
        }
    }else{
        log_it(L_NOTICE,"Результат возвращается без исправления плохих ячеек, кто это делает должен понимать, что он делает");
    }

    if (a_flags & DRS_CAL_APPLY_Y_SPLASHS){
        log_it(L_INFO,"Пробуем найти всплески по пороговому уровню %f...", a_treshold);
        for(unsigned ch=0;ch< DRS_CHANNELS_COUNT; ch++) {
            for(unsigned l_cell_id=0;l_cell_id<l_cells_proc_count;l_cell_id++){
                if (l_found_smth[ch][l_cell_id+2] ){ // проверяем, вдруг эта ячейка уже и так исключена
                    l_cell_id += 2;
                    continue;
                }

                double y[] = { a_Y[DRS_IDX(ch,l_cell_id)],
                                 a_Y[DRS_IDX(ch,l_cell_id + 1)],
                                 a_Y[DRS_IDX(ch,l_cell_id + 2)]};

                double dY[] ={ fabs( y[0] - y[1]) ,
                                 fabs( y[1] - y[2]) };


                if(
                    dY[0] > a_treshold && // Сравниваем dY он должен превышать барьер
                    dY[1] > a_treshold &&
                   //fabs(dY[0] - dY[1] ) < a_treshold/2.0 && // тут не должен превышать половину барьера

                      (   // тут проверяем форму всплеска по сути
                        ( y[1] > y[0] && y[1] > y[2] ) ||
                        ( y[1] < y[0] && y[1] < y[2] )
                      )
                    ){
                    //l_splash_id = (l_cell_id +1 ) & DRS_BANK_MASK ;
                    log_it(L_NOTICE, "Found splash for %d channel in %d cell ( dY_0 = %f, dY_1 = %f )",ch, l_cell_id+1,
                           dY[0], dY[1]); //(l_cell_id+ a_drs->shift_bank)&1023);
                    l_found_smth[ch][l_cell_id+1]  = true;
                    l_cell_id++;
                }

            }

        }
    }

    for(unsigned ch=0;ch<DRS_CHANNELS_COUNT;ch++){
        for(unsigned n = 1; n < l_cells_proc_count+1; n++){
            if(l_found_smth[ch][n]){
                unsigned idx = DRS_IDX(ch,n);

                 a_Y[idx] = (a_Y[DRS_IDX(ch, n - 1)] +
                         a_Y[DRS_IDX(ch, n + 1)]) / 2.0;
                /*if(l_cell_id_fix == 0){
                    a_Y[DRS_IDX_BANK(ch,b,0)] = (a_Y[DRS_IDX_BANK(ch,b,0)] + a_Y[DRS_IDX_BANK(ch,b,1)] ) / 2.0;
                }

                if(l_cell_id_fix >= 1022){
                    a_Y[DRS_IDX_BANK(ch,b,0)] = (a_Y[DRS_IDX_BANK(ch,b,1023)] + a_Y[DRS_IDX_BANK(ch,b,1022)] ) / 2.0;
                }*/
            }
        }
    }

}

