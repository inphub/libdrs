#include <assert.h>
#include <unistd.h>
#include <dap_common.h>
#include <math.h>

#include "data_operations.h"
#include "commands.h"


#include "drs.h"
#include "drs_data.h"
#include "drs_ops.h"
#include "drs_cal.h"
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

static unsigned int s_fin_collect(drs_t * a_drs,  drs_cal_args_t * a_args, bool a_ch9_only);
static unsigned int s_channels_calibration(drs_t * a_drs , drs_cal_args_t * a_args);

static void s_calc_coeff_b( struct amp_context * a_ctx, unsigned a_iteration, unsigned a_repeats, bool a_ch9_only);
static void s_collect_stats_b(drs_t * a_drs,struct amp_context * a_ctx, unsigned a_repeat_iter, bool a_ch9_only);

static void s_get_coefficients(drs_t * a_drs, drs_cal_args_t * a_args,  struct amp_context * a_ctx, bool a_ch9_only);
static void s_find_splash(drs_t * a_drs, double*a_Y, unsigned int a_lvl, bool a_ch9_only);
static void s_remove_splash(drs_t * a_drs, double* a_Y, bool a_ch9_only);

static bool s_debug_more = true;

/**
 * @brief drs_cal_amp
 * @param a_drs_num
 * @param a_args
 * @param a_progress
 * @return
 */
int drs_cal_amp( int a_drs_num, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress)
{
    if (a_drs_num == -1){
        int l_ret = -10;
        log_it(L_INFO, "Amplitude calibration for all DRS");
        for (size_t i=0; i < DRS_COUNT; i++ ){
            l_ret = s_proc_drs(g_drs +i, a_args,a_progress);
            if ( l_ret != 0){
                log_it(L_ERROR,"Can't run calibrate amplitude for DRS #%u code %d ",i, l_ret);
                break;
            }
        }
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

/*
 * Применяет амплитудную калибровку к данным
 * unsigned short *buffer			массив данных;
 * double *dBuf 					массив данных с результатом применения амплитудной калибровки
 * unsigned int shift 				сдвиг получаемый через getShiftIndex;
 * coefficients *coef				структура с кэффициентами;
 * unsigned int chanalLength		длинна массива для 1 канала;
 * unsigned int chanalCount			количество каналов
 * unsigned int a_flags	    		0 бит- применение калибровки для ячеек, 1 бит- межканальная калибровка,2 бит- избавление от всплесков, 3 бит- приведение к физическим виличинам
 * */
void drs_cal_ampl_apply(drs_t * a_drs, unsigned short *a_in,double *a_out, int a_flags)
{
    unsigned int l_ch_id,l_cell_id,koefIndex;

    // Если мы сейчас в режиме 9ого канала, то автоматически взводим этот флаг
    if (drs_get_mode(a_drs->id) == DRS_MODE_CAL_TIME)
        a_flags |= DRS_CAL_AMPL_CH9_ONLY;

    unsigned l_cells_proc_count = a_flags & DRS_CAL_AMPL_CH9_ONLY ? DRS_CELLS_COUNT_BANK : DRS_CELLS_COUNT_CHANNEL;
    //double * l_bi = s_bi; // a_drs->coeffs.b
    //double * l_ki = s_ki; // a_drs->coeffs.k
    //double  **l_bi = a_drs->coeffs.b;
    //double  **l_ki = a_drs->coeffs.k;
    //double average[4];
    //getAverageInt(average,buffer,DRS_CELLS_COUNT_CHANNEL,DRS_CHANNELS_COUNT);
    for(l_ch_id=0; l_ch_id<DRS_CHANNELS_COUNT;l_ch_id++){
        if(a_flags & DRS_CAL_AMPL_CH9_ONLY)
            l_ch_id = DRS_CHANNEL_9;

        for(l_cell_id=0; l_cell_id<l_cells_proc_count;l_cell_id++){
            size_t l_cell_id_masked = a_flags & DRS_CAL_AMPL_CH9_ONLY ? l_cell_id :
                                                                        l_cell_id&3072 ;
            unsigned l_inout_id = l_cell_id * DRS_CHANNELS_COUNT + l_ch_id;

            koefIndex=  a_flags & DRS_CAL_AMPL_CH9_ONLY ? (a_drs->shift +(l_cell_id &1023) ) & 1023 :
                                                          (a_drs->shift + l_cell_id ) & 1023 ;
            a_out[l_inout_id] = a_in[l_inout_id];
            if((a_flags & DRS_CAL_AMPL_APPLY_CELLS)!=0){
                double l_bi, l_ki;
                if (a_flags & DRS_CAL_AMPL_CH9_ONLY){
                    l_bi = a_drs->coeffs.b9 [koefIndex | l_cell_id_masked];
                    l_ki = a_drs->coeffs.k9 [koefIndex | l_cell_id_masked];
                }else{
                    l_bi = a_drs->coeffs.b[l_ch_id][koefIndex | l_cell_id_masked];
                    l_ki = a_drs->coeffs.k[l_ch_id][koefIndex | l_cell_id_masked];
                }


                a_out[l_inout_id] =  ( a_out[l_inout_id] - l_bi ) /
                                                 (l_ki +1.0 );
            }
            if((a_flags & DRS_CAL_AMPL_APPLY_INTERCHANNEL)!=0){
                a_out[l_inout_id] = (a_out[l_inout_id] - a_drs->coeffs.chanB[l_ch_id] ) / a_drs->coeffs.chanK[l_ch_id];
            }
            if((a_flags & DRS_CAL_AMPL_APPLY_PHYS)!=0){
                a_out[l_inout_id]=(a_out[l_inout_id]-g_ini->fastadc.adc_offsets[l_ch_id])/g_ini->fastadc.adc_gains[l_ch_id];
            }
        }
        if(a_flags & DRS_CAL_AMPL_CH9_ONLY)
            break;
    }
    if((a_flags& DRS_CAL_AMPL_APPLY_SPLASHS)!=0)
    {
        s_remove_splash(a_drs, a_out, a_flags & DRS_CAL_AMPL_CH9_ONLY);
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

    unsigned l_dac_shifts_old =  drs_adc_shift_input_get(a_drs->id);

    drs_mode_t l_mode_old = drs_get_mode(a_drs->id);
    log_it(L_INFO, "Calibrate amplitude start: count=%d, begin=%f, end=%f, mode_old=%d",a_args->param.ampl.repeats ,
           l_levels[0], l_levels[1], l_mode_old);


    drs_set_sinus_signal(false); // Выключаяем сигнал синусоиды
    drs_set_mode(a_drs->id, DRS_MODE_CAL_AMPL ); // Включаем режим калибровки амплитуды
    set_gains_drss(32, 32, 32, 32);
    start_amplifier(1);

    if( s_fin_collect( a_drs, a_args,false) !=0 ) {
        log_it(L_INFO, "No success with fin collect");
        if (a_progress) *a_progress +=5;

        l_ret = -1;
        goto lb_exit;
    }

    log_it(L_NOTICE, "Calibrate fin end: count=%d, begin=%f, end=%f, shifts=%p",a_args->param.ampl.repeats, l_levels[0], l_levels[1], l_shifts);

    if( s_channels_calibration( a_drs, a_args) !=0 ) {
        log_it(L_INFO, "No success with channels calibration");
        l_ret = -2;
        goto lb_exit;
    }

    log_it(L_NOTICE, "Calibrate 9 channel");
    // Калибруем 9ый канал
    drs_set_mode(a_drs->id, DRS_MODE_CAL_TIME);
    set_gains_drss(32, 32, 32, 32);
    start_amplifier(1);
    drs_start(a_drs->id);
    drs_dac_shift_input_set( a_drs->id, l_dac_shifts_old);

    if( s_fin_collect( a_drs, a_args,true) !=0 ) {
        log_it(L_INFO, "No success with fin collect");
        if (a_progress) *a_progress +=5;

        l_ret = -9;
        goto lb_exit;
    }


    drs_set_mode(a_drs->id, l_mode_old ); // Выключаем режим калибровки амплитуды

    if (a_progress) *a_progress +=10;
    log_it(L_NOTICE, "Channels calibrate ends: count=%d, begin=%f, end=%f, shifts=%p", a_args->param.ampl.repeats
           , l_levels[0], l_levels[1], l_shifts);

    a_drs->coeffs.indicator|=1;
    if (a_progress) *a_progress +=10;

    return l_ret;
lb_exit:
    drs_set_mode(a_drs->id, l_mode_old ); // Выключаем режим калибровки амплитуды

    drs_dac_shift_input_set( a_drs->id, l_dac_shifts_old);
    return l_ret;


}

/**
 * @brief s_fin_collect
 * @param a_drs
 * @param a_args
 * @return
 */
static unsigned int s_fin_collect( drs_t * a_drs, drs_cal_args_t * a_args, bool a_ch9_only)
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
    double shiftDACValues[DRS_CHANNELS_COUNT ];
    double dh=0.0,lvl=0.0;
    double * calibLvl = a_args->param.ampl.levels;
    unsigned l_count = a_args->param.ampl.repeats +2;
    unsigned l_cells_count = a_ch9_only ? DRS_CELLS_COUNT : DRS_CELLS_COUNT_BANK;

    struct amp_context l_ctx = {
      .average_count =       DRS_CHANNELS_COUNT * l_count,
      .average =          DAP_NEW_SIZE(   double,  DRS_CHANNELS_COUNT * l_count * sizeof(*l_ctx.average)),

      .cells_count     =     DRS_CELLS_COUNT,
      .cells =     DAP_NEW_Z_SIZE( unsigned short, DRS_CELLS_COUNT * sizeof (unsigned short) ),

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
    for(unsigned i = 0; i < l_count; i++){
        debug_if(s_debug_more, L_INFO, "Repeat #%u", i);
        lvl = calibLvl[0]+dh*((double)i);
        fill_array(shiftDACValues, &lvl, DRS_CHANNELS_COUNT, sizeof(lvl));
        drs_dac_shift_set_all(a_drs->id, shiftDACValues,g_ini->fastadc.dac_gains, g_ini->fastadc.dac_offsets);

        for(unsigned k=0;k< a_args->param.ampl.N;k++){
            if(drs_data_get(a_drs,DRS_OP_FLAG_CALIBRATE, l_ctx.cells, l_cells_count)!=0){
                log_it(L_ERROR, "data not read on %u::%u iteration", i,k);
                l_ret = -1;
                goto lb_exit;
            }
            if(a_drs->shift>1023){
               log_it(L_ERROR, "shift index went beyond on %u::%u iteration", i,k);
               l_ret = -2;
               goto lb_exit;
            }
            s_collect_stats_b(a_drs,&l_ctx,i, a_ch9_only);
            l_repeats++;
        }

        // Немного дополнительного отладочного вывода
        if( s_debug_more){
            for(unsigned n = 0; n < 10; n++){
                log_it(L_INFO, "cells[%u]=%u", n, l_ctx.cells[n]);
            }
            for(unsigned c = 0; c < DRS_CHANNELS_COUNT; c++){
                for(unsigned n = 0; n < 10; n++){
                    log_it(L_INFO, "acc[%u][%u][%u]=%f", i,c,n, l_ctx.acc[i][c][n]);
                }
            }
        }
        s_calc_coeff_b(&l_ctx, i, a_args->param.ampl.N, a_ch9_only );
        //debug_if(s_debug_more, L_DEBUG, "acc[]={%f,%f,%f,%f...}", l_ctx.acc[0], l_ctx.acc[1], l_ctx.acc[2], l_ctx.acc[3]);
    }
    //debug_if(s_debug_more, L_DEBUG, "acc[]={%f,%f,%f,%f...}", l_ctx.acc[0], l_ctx.acc[1], l_ctx.acc[2], l_ctx.acc[3]);

    for(unsigned i = 0; i < l_count * DRS_CHANNELS_COUNT; i++){
        debug_if(s_debug_more, L_DEBUG, "average[%u]=%f", i, l_ctx.average[i]);
    }
    debug_if(s_debug_more, L_INFO, "Collected stats in %u repeats", l_repeats);

    s_get_coefficients(a_drs, a_args, &l_ctx, a_ch9_only);

    log_it(L_NOTICE,"Collected coeficients in %u repeats",l_repeats);

    // Восстанавливаем уровни
    //lvl = (calibLvl[1]-calibLvl[0])/2.0 + calibLvl[0];
    //fill_array(shiftDACValues, &lvl, DRS_DAC_COUNT, sizeof(lvl));
    //drs_dac_shift_set_all(a_drs->id, shiftDACValues,g_ini->fastadc.dac_gains, g_ini->fastadc.dac_offsets);

lb_exit:
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
static unsigned int s_channels_calibration(drs_t * a_drs , drs_cal_args_t * a_args)
{
    unsigned l_count = a_args->param.ampl.repeats+2;
    double dh,shiftDACValues[DRS_CHANNELS_COUNT],l_lvl,average[DRS_CHANNELS_COUNT*l_count],l_d_buf[DRS_CELLS_COUNT], xArr[l_count],yArr[l_count];
    unsigned short l_cells[DRS_CELLS_COUNT];
    double *calibLvl = a_args->param.ampl.levels;
    unsigned int t=0,i;
    dh=(calibLvl[1]-calibLvl[0])/(l_count-1);
    log_it(L_INFO,"--Channel calibration--");
    for(t=0;t<l_count;t++){
        l_lvl=calibLvl[0]+dh*t;
        fill_array(shiftDACValues,&l_lvl,DRS_DAC_COUNT,sizeof(l_lvl));
        drs_dac_shift_set_all(a_drs->id, shiftDACValues,g_ini->fastadc.dac_gains,g_ini->fastadc.dac_offsets);
        if (drs_data_get_all(NULL,0, l_cells ) != 0){
            log_it(L_ERROR,"data not read on iteration %u", t);
            return -1;
        }
        if(a_drs->shift>1023){
            log_it(L_ERROR,"shift index went beyond on iteration %u", t);
            return -2;
        }
        drs_cal_ampl_apply(a_drs, l_cells,l_d_buf, DRS_CAL_AMPL_APPLY_SPLASHS);
        getAverage(&average[t*DRS_CHANNELS_COUNT],l_d_buf,DRS_CELLS_COUNT_CHANNEL,DRS_CHANNELS_COUNT);
    }
    s_find_splash( a_drs, l_d_buf,100,false);
    for(i=0;i<DRS_CHANNELS_COUNT;i++){
        for(t=0;t<l_count;t++){
            xArr[t]=average[DRS_CHANNELS_COUNT*t+i];
            yArr[t]=average[DRS_CHANNELS_COUNT*t+i]-(0.5-calibLvl[0]-dh*t)*DRS_CELLS_COUNT;
        }
        getCoefLine(yArr,xArr, a_args->param.ampl.repeats, &a_drs->coeffs.chanB[i],&a_drs->coeffs.chanK[i]);
    }
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

            l_rotate_index=a_ch9_only? (a_drs->shift + l_cell_id) & 1023:
                                       (a_drs->shift + (l_cell_id&1023)) & 1023 ;

            unsigned l_acc_index =  a_ch9_only?  l_rotate_index :
                                                (l_cell_id&3072) | l_rotate_index;
            unsigned l_cell_id_shift = l_cell_id * DRS_CHANNELS_COUNT + l_ch_id ;
            assert( l_acc_index< l_cells_proc_count);

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
 * @brief s_find_splash
 * @param a_drs
 * @param Y
 * @param lvl
 */
static void s_find_splash(drs_t * a_drs, double*a_Y, unsigned int a_lvl, bool a_ch9_only)
{
    unsigned int i = 0, j = 0;
    unsigned int * l_splash = a_drs->coeffs.splash;
    for(j=0;j< DRS_CHANNELS_COUNT; j++) {
        l_splash[j] = 1024;
        for(i=1;i<1000;i++){
            if(absf(a_Y[(i+1)*4+j]-a_Y[i*4+j])>a_lvl && absf(a_Y[(i-1)*4+j]-a_Y[i*4+j])>a_lvl){
                log_it(L_DEBUG, "find splash for %d channel in %d cell\n",j+1,(i+ a_drs->shift)&1023);
                l_splash[j] = ( i + a_drs->shift ) & 1023;
                break;
            }
        }
        if(absf(a_Y[1*4+j]-a_Y[j])>a_lvl && absf(a_Y[1023*4+j]-a_Y[j])>a_lvl){
            l_splash[j] = 0;
            log_it(L_DEBUG,"find splash for %u channel in %u cell",j+1,a_drs->shift);
        }
    }
}


/**
 * @brief s_remove_splash
 * @param Y
 * @param shift
 * @param coef
 */
static void s_remove_splash(drs_t * a_drs, double* a_Y, bool a_ch9_only)
{
    size_t i=0,j=0;
    for(j=0;j<DRS_CHANNELS_COUNT;j++){
        #define A_Y_IDX(a) ( (a)*DRS_CHANNELS_COUNT+j)
        i = ( a_drs->coeffs.splash[j] - a_drs->shift + 1023) & 1023;
        //if((i > 0) && (i < 1023)){
        //    a_Y[A_Y_IDX(i)] = (a_Y[A_Y_IDX(i+1)] + a_Y[a_Y[A_Y_IDX(i-1)]) / 2;
        //}

        if(i == 0){
            a_Y[j] = (a_Y [4*1 + j] + a_Y[4*2 + j]) / 2;
        }

        if(i == 1023){
            a_Y[1023 * 4 + j]=(a_Y[j] + a_Y[1022 * 4 + j]) / 2;
        }
    }
}
