#include <assert.h>
#include <unistd.h>
#include <dap_common.h>

#include "data_operations.h"

#include "drs.h"
#include "drs_data.h"
#include "drs_cal.h"
#include "drs_cal_amp.h"

#define LOG_TAG "drs_cal_amp"

static int s_proc_drs( drs_t * a_drs, drs_cal_args_t * a_args, atomic_uint_fast32_t * a_progress);

static unsigned int s_fin_collect(drs_t * a_drs,  drs_cal_args_t * a_args);
static unsigned int s_channels_calibration(drs_t * a_drs , drs_cal_args_t * a_args);

static void s_calc_coeff_b(double *a_acc,unsigned int *a_stats, double *a_average);
static void s_collect_stats_cells(drs_t * a_drs, double *a_acc,unsigned short *a_values,unsigned int *a_stats);

static void s_get_coefficients(drs_t * a_drs, drs_cal_args_t * a_args,  double *a_acc, double *a_average);
static void s_find_splash(drs_t * a_drs, double*a_Y, unsigned int a_lvl);
static void s_remove_splash(drs_t * a_drs, double* a_Y);

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
void drs_cal_ampl_apply(drs_t * a_drs, unsigned short *buffer,double *dBuf, int a_flags)
{
    unsigned int j,k,koefIndex;
    double average[4];
    getAverageInt(average,buffer,1000,4);
    for(j=0;j<DRS_CHANNELS_COUNT;j++)
    {
        for(k=0;k<1024;k++)
        {
            koefIndex=( a_drs->shifts[k>>1] )&1023;
            dBuf[k*DRS_CHANNELS_COUNT+j]=buffer[k*DRS_CHANNELS_COUNT+j];
            if((a_flags & DRS_CAL_AMPL_APPLY_CELLS)!=0)
            {
                dBuf[k*DRS_CHANNELS_COUNT+j]=(dBuf[k*DRS_CHANNELS_COUNT+j]-a_drs->coeffs.b[koefIndex*4+j])/(a_drs->coeffs.k[koefIndex*4+j]+1);
            }
            if((a_flags & DRS_CAL_AMPL_APPLY_INTERCHANNEL)!=0)
            {
                dBuf[k*DRS_CHANNELS_COUNT+j]-= a_drs->coeffs.chanB[j]+a_drs->coeffs.chanK[j]*average[j];
            }
            if((a_flags & DRS_CAL_AMPL_APPLY_PHYS)!=0)
            {
                dBuf[k*DRS_CHANNELS_COUNT+j]=(dBuf[k*DRS_CHANNELS_COUNT+j]-g_ini->fastadc.adc_offsets[j])/g_ini->fastadc.adc_gains[j];
            }
        }
    }
    if((a_flags& DRS_CAL_AMPL_APPLY_SPLASHS)!=0)
    {
        s_remove_splash(a_drs, dBuf);
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
    log_it(L_INFO, "Calibrate amplitude start: count=%d, begin=%f, end=%f, shifts=%p",a_args->param.ampl.repeats ,
           l_levels[0], l_levels[1], l_shifts);
    if(s_fin_collect( a_drs, a_args)==0)
    {
        log_it(L_INFO, "No success with fin collect");
        if (a_progress) *a_progress +=10;

        return -1;
    }else{
        log_it(L_NOTICE, "Calibrate fin end: count=%d, begin=%f, end=%f, shifts=%p",a_args->param.ampl.repeats, l_levels[0], l_levels[1], l_shifts);
        if(s_channels_calibration( a_drs, a_args)==0)
        {
            log_it(L_INFO, "No success with channels calibration");
            return -2;
        }
        if (a_progress) *a_progress +=10;
        log_it(L_NOTICE, "Channels calibrate ends: count=%d, begin=%f, end=%f, shifts=%p", a_args->param.ampl.repeats
               , l_levels[0], l_levels[1], l_shifts);
        drs_dac_shift_set_all(l_shifts,g_ini->fastadc.dac_gains, g_ini->fastadc.dac_offsets);
        drs_dac_set(1);
        a_drs->coeffs.indicator|=1;
        if (a_progress) *a_progress +=10;

        return 0;
    }
}

/**
 * @brief s_fin_collect
 * @param a_drs
 * @param a_args
 * @return
 */
static unsigned int s_fin_collect( drs_t * a_drs, drs_cal_args_t * a_args)
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
    unsigned int i=0,k=0;
    double shiftDACValues[DRS_DCA_COUNT];
    double dh=0.0,lvl=0.0;
    double * calibLvl = a_args->param.ampl.levels;
    unsigned count = a_args->param.ampl.repeats;

    double * l_average =          DAP_NEW_SIZE(   double,         DRS_DCA_COUNT*a_args->param.ampl.repeats* sizeof(double) );
    unsigned short *l_cells =     DAP_NEW_Z_SIZE( unsigned short, DRS_PAGE_ALL_SIZE * DRS_CHANNELS_COUNT                   );
    unsigned int *l_stats =       DAP_NEW_SIZE(   unsigned int,   DRS_CELLS_COUNT_ALL * sizeof (*l_stats)                  );
    double *l_acc =               DAP_NEW_Z_SIZE( double,         count * DRS_CELLS_COUNT_ALL * sizeof (double)            );

    memset(&a_drs->coeffs.b,0 , sizeof (a_drs->coeffs.b));
    memset(&a_drs->coeffs.k,0 , sizeof (a_drs->coeffs.k));


    drs_mode_set(1);

    dh=(calibLvl[1]-calibLvl[0])/((double) (a_args->param.ampl.repeats-1));

    log_it(L_NOTICE,"--Collecting coeficients in %u iterations, step length %f", a_args->param.ampl.repeats, dh);

    for(i = 0; i < a_args->param.ampl.repeats; i++){
        lvl = calibLvl[0]+dh*((double)i);
        memset(l_stats, 0, DRS_CELLS_COUNT_ALL * sizeof (*l_stats) );
        fill_array(shiftDACValues, &lvl, DRS_DCA_COUNT, sizeof(lvl));

        drs_dac_shift_set_all(   shiftDACValues,g_ini->fastadc.dac_gains, g_ini->fastadc.dac_offsets);
        drs_dac_set(1);

        for(k=0;k< a_args->param.ampl.N +   2 ;k++){
            if(drs_data_get_all(a_drs,0, l_cells)!=0){
                log_it(L_ERROR, "data not read on %u::%u iteration", i,k);
                drs_mode_set(0);
                goto lb_exit;
            }
            if(a_drs->shift>1023)
            {
               log_it(L_ERROR, "shift index went beyond on %u::%u iteration", i,k);

               goto lb_exit;
            }
            s_collect_stats_cells(a_drs, l_acc, l_cells, l_stats);
        }
        s_calc_coeff_b(l_acc,l_stats,l_average);
    }
    drs_mode_set(0);
    s_get_coefficients(a_drs, a_args, l_acc, l_average);

    log_it(L_NOTICE,"Collected coeficients in %u::%u iterations",i,k);

    l_ret = 1;
lb_exit:
    DAP_DELETE(l_stats);
    DAP_DELETE(l_cells);
    DAP_DELETE(l_acc);
    DAP_DELETE(l_average);

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
    unsigned count = a_args->param.ampl.repeats;
    double dh,dn,shiftDACValues[4],lvl,average[4*count],dBuf[16384], xArr[count],yArr[count];
    unsigned short loadData[16384];
    double *calibLvl = a_args->param.ampl.levels;
    unsigned int t=0,i;
    dh=(calibLvl[1]-calibLvl[0])/(count-1);
    log_it(L_INFO,"--Channel calibration--");
    for(t=0;t<count;t++)
    {
        lvl=calibLvl[0]+dh*t;
        fill_array((unsigned char*)(shiftDACValues),(unsigned char*)&lvl,4,sizeof(dn));
        drs_dac_shift_set_all(shiftDACValues,g_ini->fastadc.dac_gains,g_ini->fastadc.dac_offsets);
        usleep(200);
        drs_dac_set(1);
        usleep(200);
        if (drs_data_get_all(NULL,0, loadData ) != 0){
            log_it(L_ERROR,"data not read on iteration %u", t);
            drs_mode_set(0);
            return 0;
        }
        if(a_drs->shift>1023){
            log_it(L_ERROR,"shift index went beyond on iteration %u", t);
            return 0;
        }
        drs_cal_ampl_apply(a_drs, loadData,dBuf, DRS_CAL_AMPL_APPLY_SPLASHS);
        getAverage(&average[t*4],dBuf,1000,4);
    }
    s_find_splash( a_drs, dBuf,100);
    for(i=0;i<4;i++)
    {
        for(t=0;t<count;t++)
        {
            xArr[t]=average[4*t+i];
            yArr[t]=average[4*t+i]-(0.5-calibLvl[0]-dh*t)*16384;
        }
        getCoefLine(yArr,xArr, a_args->param.ampl.repeats, &a_drs->coeffs.chanB[i],&a_drs->coeffs.chanK[i]);
    }
    return 1;
}


/**
 * @brief s_get_coefficients   Вычисляет коэфициенты
 * @param a_drs                Объект DRS
 * @param a_args               Аргументы калибровки
 * @param a_acc                Сумма знчаений для ячейки
 * @param a_average            Средние значения по каналам
 */
static void s_get_coefficients(drs_t * a_drs, drs_cal_args_t * a_args,  double *a_acc, double *a_average)
{
    unsigned int i,j,k;
    unsigned int l_repeats = a_args->param.ampl.repeats;
    double * yArr = l_repeats ? DAP_NEW_STACK_SIZE(double, sizeof(double) * l_repeats) : NULL;
    double * xArr = l_repeats ? DAP_NEW_STACK_SIZE(double, sizeof(double) * l_repeats): NULL;
    double * l_levels = a_args->param.ampl.levels;
    double dh;
        dh=(l_levels[1]-l_levels[0])/(l_repeats-1);
        for(j=0;j<4;j++)
        {
                for(i=0;i<1024;i++)
                {
                        for(k=0;k<l_repeats;k++)
                        {
                                xArr[k]=(0.5-l_levels[0]-dh*k)*16384;
                                yArr[k]=a_acc[k*8192+i*4+j]-a_average[k*4+j];
                        }
                        getCoefLine(yArr,xArr,l_repeats,&a_drs->coeffs.b[i*4+j],&a_drs->coeffs.k[i*4+j]);
                }
        }
        for(j=0;j<4;j++)
        {
                for(k=0;k<l_repeats;k++)
                {
                        yArr[k]= a_average[k*4+j];
                        xArr[k]=(0.5-l_levels[0]-dh*k)*16384;
                }
        }
}

/**
 * @brief s_collect_stats_cells         Собирает статистику для каждой ячкейки
 * @param a_drs                         Объект DRS
 * @param a_acc                         сумма знчаений для ячейки
 * @param a_buff                        массив данных
 * @param a_statistic                   статистика для ячеек
 */
static void s_collect_stats_cells(drs_t * a_drs, double *a_acc,unsigned short *a_values,unsigned int *a_stats)
{
	unsigned int j,k,l_rotate_index;
		for(j=0;j<4;j++)
		{
			for(k=0;k<1000;k++)
			{
				l_rotate_index=( a_drs->shift+k)&1023;
				a_acc[l_rotate_index*4+j]+=a_values[k*4+j];
				a_stats[l_rotate_index*4+j]++;
			}
		}
}

/**
 * @brief s_calc_coeff_b  Вычисляет коэффиценты для амплитудной калибровки ячеек
 * @param a_acc
 * @param a_stats
 * @param a_average
 */
static void s_calc_coeff_b(double *a_acc,unsigned int *a_stats, double *a_average)
{
	unsigned int k,j;
	double val=0;
	for(j=0;j<DRS_DCA_COUNT;j++)
	{
		a_average[j]=0;
		for(k=0;k<1024;k++)
		{
			val=a_acc[k*4+j]/a_stats[k*4+j];
			a_average[j]+=val;
			a_acc[k*4+j]=val;
		}
		a_average[j]=a_average[j]/1024;

	}
}

/**
 * @brief s_find_splash
 * @param a_drs
 * @param Y
 * @param lvl
 */
static void s_find_splash(drs_t * a_drs, double*a_Y, unsigned int a_lvl)
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
static void s_remove_splash(drs_t * a_drs, double* a_Y)
{
    size_t i=0,j=0;
    for(j=0;j<DRS_CHANNELS_COUNT;j++){
        i = ( a_drs->coeffs.splash[j] - a_drs->shift + 1023) & 1023;
        if((i > 0) && (i < 1023)){
            a_Y[i*4+j] = (a_Y[(i + 1) * 4 + j] + a_Y[(i - 1) * 4 + j]) / 2;
        }

        if(i == 0){
            a_Y[j] = (a_Y [4*1 + j] + a_Y[4*2 + j]) / 2;
        }

        if(i == 1023){
            a_Y[1023 * 4 + j]=(a_Y[j] + a_Y[1022 * 4 + j]) / 2;
        }
    }
}
