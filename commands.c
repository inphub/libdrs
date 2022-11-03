/*
 * commands.c
 *
 *  Created on: 20 дек. 2019 г.
 *      Author: Denis
 */
#include <stdio.h>
#include <unistd.h>

#include "mem_ops.h"
#include "drs_ops.h"
#include "commands.h"

/**
 * double *shiftDAC		сдвиги с фронтпанели;
 * float *DAC_gain		массив из ini
 * float *DAC_offset	массив из ini
 */
void setShiftAllDac(double *shiftDAC,float *DAC_gain,float *DAC_offset)//fix
{
	int i;
	unsigned short shiftDACValues[4];
	for(i=0;i<4;i++)
	{
		shiftDACValues[i]=(shiftDAC[i]*DAC_gain[i]+DAC_offset[i]);
		//printf("shiftDAC[%d]=%f\tshiftDACValues[%d]=%d\n",i,shiftDAC[i],i,shiftDACValues[i]);
	}
	setAllDAC(shiftDACValues);
	setDAC(1);
}

/**
 * unsigned short *shiftValue 		масиив сдивгов для ЦАП
 */
void setAllDAC(unsigned short *shiftValue)//fix
{
	int j;
	for(j=0;j<2;j++)
	{
		setDACInputShift(j,((shiftValue[j*2]<<16)&0xFFFF0000)|shiftValue[j*2+1]);
	}
}

void setNumPages(unsigned int num)
{
    write_reg(DRS1_NUM_PAGE,num);
    write_reg(DRS2_NUM_PAGE,num);
	usleep(100);
}

void setSizeSamples(unsigned int num)
{
	write_reg(0x19,num);
	usleep(100);
}

void drs_set_mode(unsigned int mode)
{
	write_reg(DRS_MODE_REG, mode);
	usleep(100);
    drs_cmd(-1, INIT_DRS);
}

void setDACInputShift(unsigned int addrShift,unsigned int value)//fix
{
	write_reg(0x8+addrShift,value);
	usleep(100);
}

void setDAC(unsigned int onAH)//fix
{
	//unsigned int onAH=1,dacSelect=2;
	write_reg(0x07,(onAH&1));
	usleep(200);
}

unsigned int readEnWrite(unsigned int drsmask)
{
    unsigned int res;
	usleep(100);
//    printf("read: adr=0x%08x, val=0x%08x\n", DRS_MODE_REG, mode), fflush(stdout);
    res=read_reg(49);//(read_reg(49)&(drsmask&1))&(read_reg(50)&((drsmask>>1)&1));
    //printf("readEnWrite=%d\n", res);
 	return (res);
}

unsigned int readExternalStatus (unsigned int stat)
{
	usleep(100);
	return read_reg(0x30|(0xf & stat));
}


void set_dma_addr_drs1(unsigned int offs)
{
	 	write_reg(0x00000017, offs);
}

void set_size_dma_drs1(unsigned int sz)
{
	 	write_reg(0x00000019, sz);
}

void set_dma_addr_drs2(unsigned int offs)
{
	 	write_reg(0x00000018, offs);
}

void set_size_dma_drs2(unsigned int sz)
{
	 	write_reg(0x0000001a, sz);
}

void set_shift_addr_drs1(unsigned int offs)
{
		write_reg(0x0000001c, offs);
}

void set_shift_addr_drs2(unsigned int offs)
{
		write_reg(0x0000001d, offs);
}

void clk_select(unsigned int val)
{
		write_reg(0x00000004, val);
}

void clk_phase(unsigned int val)
{
		write_reg(0x00000006, val);
}

void clk_select_internal_value(unsigned int val)
{
		write_reg(0x0000001e, val);
}

void clk_start(unsigned int st)
{
		write_reg(0x00000005, st);
}

void set_dac_offs_drs1(unsigned short valch_a, unsigned short valch_b)
{
		write_reg(0x00000008, (((unsigned int)valch_b)<<16)|((unsigned int)valch_a));
}

void set_dac_offs_drs2(unsigned short valch_c, unsigned short valch_d)
{
		write_reg(0x00000009, (((unsigned int)valch_d)<<16)|((unsigned int)valch_c));
}

void start_dac(unsigned int st)
{
		write_reg(0x00000007, st);
}

void set_dac_rofs_O_ofs_drs1(unsigned short rofs, unsigned short O_ofs)
{
		write_reg(0x0000000a, (((unsigned int)O_ofs)<<16)|((unsigned int)rofs));
}

void set_dac_speed_bias_drs1(unsigned short speed, unsigned short bias)
{
		write_reg(0x0000000b, (((unsigned int)bias)<<16)|((unsigned int)speed));
}

void set_dac_rofs_O_ofs_drs2(unsigned short rofs, unsigned short O_ofs)
{
		write_reg(0x0000000c, (((unsigned int)O_ofs)<<16)|((unsigned int)rofs));
}

void set_dac_speed_bias_drs2(unsigned short speed, unsigned short bias)
{
		write_reg(0x0000000d, (((unsigned int)bias)<<16)|((unsigned int)speed));
}

void set_dac_9ch_ofs(unsigned short speed)
{
		write_reg(0x0000001f, (unsigned int)speed);
}

void set_gains_drss(unsigned short cha, unsigned short chb, unsigned short chc, unsigned short chd)
{
		write_reg(0x00000002, (((unsigned int)chd)<<24)|(((unsigned int)chc)<<16)|(((unsigned int)chb)<<8)|(((unsigned int)cha)<<0));
}

void start_amplifier(unsigned int st)
{
		write_reg(0x00000001, st);
}

void set_starts_number_drs1(unsigned int num)
{
		write_reg(0x00000013, num);
}

void set_zap_delay_drs1(unsigned int delay)
{
		write_reg(0x00000011, delay);
}

void set_starts_number_drs2(unsigned int num)
{
		write_reg(0x00000014, num);
}

void set_zap_delay_drs2(unsigned int delay)
{
		write_reg(0x00000012, delay);
}

void set_mode_drss(unsigned int mode)
{
		write_reg(0x00000010, mode);
}

void init_drs1(void)
{
		write_reg(0x0000000e, 1);
}

void init_drs2(void)
{
		write_reg(0x0000000f, 1);
}

void start_drs1(void)
{
		write_reg(0x0000000e, 2);
}

void start_drs2(void)
{
		write_reg(0x0000000f, 2);
}



/// OLD COMMANDS
///
///



void pageMode(unsigned int on)//fix
{
/*	write_reg(16,(on&1)<<1);
    usleep(300);*/
}

/*void externalStart(unsigned int on)//fix
{
    write_reg(14,(on&1)<<2);
    write_reg(15,(on&1)<<2);
    usleep(100);
}*/

void StartDRSs(unsigned int on)//fix
{
    write_reg(14,(on&1)<<2);
    write_reg(15,(on&1)<<2);
    usleep(100);
}

void setMode(unsigned int mode)//unsigned int calibr=0x1, pachka=0<<1;
{
//0 - soft start
//1 - external start
//2 - page mode
//3 - amplitude calibrate
//4 - time calibrate
//5 - calibration chan
    write_reg(16, mode);
    usleep(100);
    write_reg(14, 1);//initDRS1
    write_reg(15, 1);//initDRS2
    usleep(100);
}

/*
void setCalibrate(unsigned int calibr,unsigned int sigsin)//unsigned int calibr=0x1, pachka=0<<1;
{
//0 - soft start
//1 - external start
//2 - page mode
//3 - amplitude calibrate
//4 - time calibrate
    write_reg(16, calibr&1);
    write_reg(sigsin&1);
    usleep(100);
}
*/

void setTimeCalibrate(unsigned int enable)//fix
{
    write_reg(16,(enable&1)<<2);
    usleep(100);
}

void setWorkDRS(unsigned int enable)//fix
{
    write_reg(14,(enable&1)<<1);
    write_reg(15,(enable&1)<<1);
    usleep(100);
}

void softStartRecorder(unsigned int enable)
{
//	write_reg(14, enable&1);
//	write_reg(15, enable&1);
//	usleep(100);
}

void flagEndRead(unsigned int enable)
{
    write_reg(21,enable&1);
    write_reg(22,enable&1);
    usleep(100);
}

