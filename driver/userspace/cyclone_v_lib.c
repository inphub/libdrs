#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>//usleep
#include <sys/ioctl.h>

#include <cyclone_v_lib.h>

int cyclone_v_regread(int fd, uint32_t offset, uint32_t *value)
{
	cyclone_v_reg_t devReg;
	int ret;

	devReg.offset = offset;
	ret = ioctl(fd, CYCLONE_V_IOC_REG_READ, &devReg);
	*value = devReg.value;
	return ret;
}

int cyclone_v_regwrite(int fd, uint32_t offset, uint32_t value)
{
	cyclone_v_reg_t devReg;

	devReg.offset = offset;
	devReg.value = value;
	return ioctl(fd, CYCLONE_V_IOC_REG_WRITE, &devReg);
}

int cyclone_v_blockread(int fd, uint8_t npage, uint8_t pages, uint32_t *data)
{
    cyclone_v_recvblock_t devRecvBlock;
    devRecvBlock.npage = npage;
    devRecvBlock.pages = pages;
    devRecvBlock.data = data;
	return ioctl(fd, CYCLONE_V_IOC_BLOCKREAD, &devRecvBlock);
}

int cyclone_v_blockwrite(int fd, uint8_t npage, uint8_t pages, uint32_t *data)
{
    cyclone_v_recvblock_t devRecvBlock;
    devRecvBlock.npage = npage;
    devRecvBlock.pages = pages;
    devRecvBlock.data = data;
	return ioctl(fd, CYCLONE_V_IOC_BLOCKWRITE, &devRecvBlock);
}

