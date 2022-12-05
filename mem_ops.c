#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "mem_ops.h"

#define PAGE_SIZE 8192
#define HPS2FPGA_BRIDGE_BASE	0xC0000000 //данные быстрых АЦП
#define LWHPS2FPGA_BRIDGE_BASE	0xff200000 //управление
#define SHIFT_DRS1	0x2FD00000
#define SHIFT_DRS2	0x3FD00000


int fd;
volatile unsigned int *control_mem;
void *control_map;
void *data_map_drs1, *data_map_drs2, *data_map_shift_drs1, *data_map_shift_drs2, *data_map;

#define LOG_TAG "mem_ops"

#define MAP_SIZE           (4096)
#define MAP_MASK           (MAP_SIZE-1)

void memw(off_t byte_addr, uint32_t data)
{
 void *map_page_addr, *map_byte_addr;
  map_page_addr = mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, byte_addr & ~MAP_MASK );
  if( map_page_addr == MAP_FAILED ) {
    perror( "mmap" );
    return;
  }
  map_byte_addr = map_page_addr + (byte_addr & MAP_MASK);
  *( ( uint32_t *) map_byte_addr ) = data;
  if( munmap( map_page_addr, MAP_SIZE ) ) {
    perror( "munmap" );
    return;
  }
}

uint32_t memr(off_t byte_addr)
{
 void *map_page_addr, *map_byte_addr;
 uint32_t data;
  map_page_addr = mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, byte_addr & ~MAP_MASK );
  if( map_page_addr == MAP_FAILED ) {
    perror( "mmap" );
    return 0;
  }
  map_byte_addr = map_page_addr + (byte_addr & MAP_MASK);
  data = *( ( uint32_t *) map_byte_addr );
  printf( "data = 0x%08x\n", data );
  if( munmap( map_page_addr, MAP_SIZE ) ) {
    perror( "munmap" );
    return 0;
  }
  return (data);
}

int init_mem(void)
{
    int ret = EXIT_FAILURE;
//	unsigned char value;
    off_t control_base = LWHPS2FPGA_BRIDGE_BASE;
    off_t data_base_drs1 = SDRAM_BASE_DRS1;
    off_t data_base_drs2 = SDRAM_BASE_DRS2;
    off_t data_shift_drs1 = SHIFT_DRS1;
    off_t data_shift_drs2 = SHIFT_DRS2;
    off_t data_base_map_offset = MEMORY_BASE;

    /* open the memory device file */
    fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (fd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    /* map the LWHPS2FPGA bridge into process memory */
    control_map = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, control_base);
    if (control_map == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_drs1 = mmap(NULL, SDRAM_SPAN_DRS1, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_base_drs1);
    if (data_map_drs1 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map = mmap(NULL, MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_base_map_offset);
    if (data_map == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_drs2 = mmap(NULL, SDRAM_SPAN_DRS2, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_base_drs2);
    if (data_map_drs2 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_shift_drs1 = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_shift_drs1);
    if (data_map_shift_drs1 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }

    data_map_shift_drs2 = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, data_shift_drs2);
    if (data_map_shift_drs2 == MAP_FAILED) {
        perror("mmap");
        goto cleanup;
    }
    ret = 0;

cleanup:
    return ret;
}

void write_reg(unsigned int reg_adr, unsigned int reg_data)
{
    /* get the delay_ctrl peripheral's base address */
    control_mem = (unsigned int *) (control_map + reg_adr*4);
    log_it(L_DEBUG, "write: adr=0x%08x, val=0x%08x", reg_adr, reg_data);

    /* write the value */
    *control_mem = reg_data;
}

unsigned int read_reg(unsigned int reg_adr)
{
    unsigned int reg_data;
    control_mem = (unsigned int *) (control_map + reg_adr*4);
    reg_data=(unsigned int)control_mem[0];
//    printf("read: adr=0x%08x, val=0x%08x\n\r", reg_adr, reg_data), fflush(stdout);
    return(reg_data);
}

void deinit_mem(void)
{
    if (munmap(control_map, PAGE_SIZE) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map, MEMORY_SIZE) < 0)
    {
        perror("munmap");
        goto cleanup;
    }

    if (munmap(data_map_drs1, SDRAM_SPAN_DRS1) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map_drs2, SDRAM_SPAN_DRS2) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map_shift_drs1, 0x1000) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
    if (munmap(data_map_shift_drs2, 0x1000) < 0)
    {
        perror("munmap");
        goto cleanup;
    }
cleanup:
    close(fd);
}
