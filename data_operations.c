/*
 * data_operations.c
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: Denis
 */
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "data_operations.h"
#include "commands.h"
#include "mem_ops.h"
#include "drs_ops.h"

/**
 * МНК
 * double*yArr			массив Y
 * double *xArr 		массив X
 * unsigned int length  длинна массивов
 * double *b			адрес переменной для записи b
 * double *k			адрес переменной для записи k
 */
void getCoefLine(double*yArr,double *xArr,unsigned int length,double *b,double *k)
{
    // length может быть 0, тогда массивы yArr и xArr будут NULL
    double x=0,x2=0,xy=0,y=0;
    unsigned int i=0;
    for(i=0;i<length;i++){
            x+=xArr[i];
            y+=yArr[i];
            xy+=xArr[i]*yArr[i];
            x2+=xArr[i]*xArr[i];
    }
    *k= ( ((double) length)*xy- x*y )/(  ((double)length)*x2 - x*x);
    *b=  (y-*k*x)/ ((double)length);
    //printf("k=%f\tb=%f\n",*k,*b);
}
void do_on_array(double *array,int length,void (*operation)(double *num)){
	int i;
	for(i=0;i<length;i++){
		(*operation)(&array[i]);
	}
}
void pageModeStart(unsigned int num)
{
	unsigned int flag=0;
	setNumPages(num);
//	setSizeSamples(num*1024);//Peter fix
//	pageMode(1);
	flag=readEnWrite()>>31;
	while(flag==0)
	{
		printf("flag=%d\n",flag);
		readExternalStatus(0x3d);
		flag=readEnWrite()>>31;
	}
    drs_set_flag_end_read(-1, true);
	readExternalStatus(0x3d);
}

/**
 * unsigned short *buffer		буфер данных;
 * unsigned int *shift			указатель на  unsigned int в который положится сдвиг;
 * unsigned short extStart		индикатор внешнего запуска;
 */
unsigned int onceGet(unsigned short *buffer,unsigned int *shift,unsigned int calibrate,unsigned int extStart,unsigned int drsnum)
{
	unsigned int flag=0,i=0,end=0;
	if(extStart==0)
	{
		setWorkDRS(1);
		usleep(100);
		if(calibrate==0)
		{
			softStartRecorder(1);
		}
	}
	flag=readEnWrite()>>31;
	while(flag==0 && end==0)
	{
		flag=readEnWrite()>>31;
		i++;
		if(extStart==0)
		{
			if(i>100)
			{
				end=1;
			}
		}else{
			//if(ext_start==0){end=1;)
		}
		//readExternalStatus(0xc); //Peter fix
	}
	if(flag==1){
        readNPage(&buffer[0],&shift[0],0,drsnum);
//		readNPage(&buffer[8192],&shift[1024],0,1,drsnum);
	}
	flagEndRead(1);
	return flag;
}

/**
 * unsigned int drsnum		номер drs для вычитывания сдвига
 * return 					индекс сдвига;
 */
unsigned int getShiftIndex(unsigned int drsnum)//npage
{
    unsigned short tmpshift;
    if (drsnum==0)
     tmpshift=((unsigned long *)data_map_shift_drs1)[0]&1023;
    else
     tmpshift=((unsigned long *)data_map_shift_drs2)[0]&1023;
	return tmpshift;
}

/**
 * double *average					масиив со средними значениями каналов;
 * double *data						массив данных
 * unsigned int chanalLength		длинна канала;
 * unsigned int chanalCount			число каналов;
 */
void getAverage(double *average,double *data,unsigned int chanalLength,unsigned int chanalCount)
{
    unsigned int i,j;
	for(i=0;i<chanalCount;i++)
	{
		average[i]=0.0;
		for(j=0;j<chanalLength;j++)
		{
			average[i]+=data[j*chanalCount+i];
		}
		average[i]/= (double) chanalLength;
		//printf("average[%d]=%f\n",i,average[i]);
	}
}

/**
 * double *         a_average		масиив со средними значениями каналов;
 * unsigned short*  a_cells		массив данных
 * unsigned int     a_ch_cells_count	длинна канала;
 * unsigned int     a_ch_count		число каналов;
 */
void getAverageInt(double *a_average,unsigned short *a_cells,unsigned int a_ch_cells_count,unsigned int a_ch_count)
{
    unsigned int i,j;
    for(i=0;i<a_ch_count;i++){
        a_average[i]=0;
        for(j=0;j<a_ch_cells_count;j++){
                a_average[i]+=a_cells[j*a_ch_count+i];
        }
        a_average[i]/= (double) a_ch_cells_count;
        //printf("average[%d]=%f\n",i,average[i]);
    }
}

 double absf(double value)
 {
	 if(value<0)
	 {
		 return value*-1;
	 }else{
		 return value;
	 }
 }

 /**
  * unsigned short *buffer		массив данных
  * unsigned int *shift			сдвиг;
  * unsigned int pageCount		номер страницы;
  */
 void readNPage(unsigned short *buffer,unsigned int* shift,unsigned int numPage, unsigned int drsnum)
 {
    if (drsnum==0)
        memcpy(buffer, &(((unsigned short *)data_map_drs1)[numPage*DRS_CELLS_COUNT_ALL*DRS_CHANNELS_COUNT]), DRS_PAGE_ALL_SIZE  );
  	else
        memcpy(buffer, &(((unsigned short *)data_map_drs2)[numPage*DRS_CELLS_COUNT_ALL*DRS_CHANNELS_COUNT]), DRS_PAGE_ALL_SIZE);
        *shift=getShiftIndex(drsnum);
 }

 /**
  * unsigned short *buffer		массив данных
  * unsigned int *shift			сдвиг;
  * unsigned int pageCount		номер страницы;
  */
 void writeNPage(unsigned short *buffer,unsigned int numPage, unsigned int drsnum)
 {
    if (drsnum==0)
 	 memcpy(&(((unsigned short *)data_map_drs1)[numPage*16384]),buffer, 0x8000);
 	else
 	 memcpy(&(((unsigned short *)data_map_drs2)[numPage*16384]),buffer, 0x8000);
 }
/**
 * unsigned short *buffer		массив данных
 * unsigned int *shift			сдвиги;
 * unsigned int pageCount		число страниц;
 */
void readNPages(unsigned short *buffer,unsigned int *shift,unsigned int pageCount, unsigned int step, unsigned int drsnum)
{
     unsigned int t;
	 for (t=0; t<pageCount;t++)
	 {
         readNPage(&buffer[t*step],&shift[t],t, drsnum);
	 }
}



