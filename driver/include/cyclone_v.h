#ifndef __CYCLONE_V__
#define __CYCLONE_V__
//#include <stdint.h>
#include <linux/types.h>
#include <linux/ioctl.h>

#ifdef __KERNEL__ /* The initial part of the file is driver-internal stuff */
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/wait.h>
#include <linux/completion.h>

#define MEM_PAGE_SEZE 1024
#define MEM_PAGE_SEZE_IN_BYTES sizeof(uint32_t)*MEM_PAGE_SEZE

void cyclone_V_write_reg(uint32_t offset, uint32_t value);
uint32_t cyclone_V_read_reg(uint32_t offset);

#endif /* __KERNEL__ */

//#define PXIE_ADC12500_KERNEL_DMA_BLOCK_SIZE 3072

typedef struct 
{
	uint32_t offset; /* offset from baseaddr */
	uint32_t value;  /* data which will be read / written */
}cyclone_v_reg_t;

typedef struct 
{
	uint8_t npage;
    uint8_t pages;
	uint32_t *data;    /* pointer to buffer */
}cyclone_v_recvblock_t;

#define CYCLONE_V_IOC_MAGIC  's'

#define CYCLONE_V_IOC_REG_READ                      _IOR(CYCLONE_V_IOC_MAGIC, 0, cyclone_v_reg_t *)
#define CYCLONE_V_IOC_REG_WRITE                     _IOW(CYCLONE_V_IOC_MAGIC, 1, cyclone_v_reg_t *)
#define CYCLONE_V_IOC_BLOCKRECV                     _IOR(CYCLONE_V_IOC_MAGIC, 2, cyclone_v_recvblock_t *)
#define CYCLONE_V_IOC_BLOCKREAD                     _IOR(CYCLONE_V_IOC_MAGIC, 3, cyclone_v_recvblock_t *)
#define CYCLONE_V_IOC_BLOCKWRITE                    _IOW(CYCLONE_V_IOC_MAGIC, 4, cyclone_v_recvblock_t *)
#define CYCLONE_V_IOC_WAIT_INT1		                _IOW(CYCLONE_V_IOC_MAGIC, 5, uint32_t *)


#define CYCLONE_V_IOC_MAXNR			10

#endif /* __CYCLONE_V__ */

