/*
 * data_operations.h
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: Denis
 */
#pragma once

#include "drs.h"

void getCoefLine(double*yArr,double *xArr,unsigned int length,double *b,double *k);
void arrayOperation(double *array,int length,void (*operation)(double *num));
void pageModeStart(unsigned int num);
unsigned int onceGet(unsigned short *buffer,unsigned int *shift,unsigned int calibrate,unsigned int extStart,unsigned int drsnum);
unsigned int onceGet1024(unsigned short *buffer,unsigned int *shift,unsigned int calibrate,unsigned int extStart,unsigned int drsnum);
unsigned int getShiftIndex(unsigned int drsnum);
double absf(double value);
void fillArray(unsigned char *array,unsigned char *value,unsigned int arrayLength,unsigned int sizeOfValue);
void getAverage(double *average,double *data,unsigned int chanalLength,unsigned int chanalCount);
void getAverageInt(double *average,unsigned short *data,unsigned int chanalLength,unsigned int chanalCount);
void readNPage(unsigned short *buffer,unsigned int *shift,unsigned int numPage, unsigned int drsnum);
void writeNPage(unsigned short *buffer,unsigned int numPage, unsigned int drsnum);
void readNPages(unsigned short *buffer,unsigned int *shift,unsigned int pageCount, unsigned int step, unsigned int drsnum);
