#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/ioctl.h>

#include "include/cyclone_v.h"

#define CYCLONE_V_REGS_BASE_ADDR 0xff202000
#define CYCLONE_V_REGS_SIZE 0x4096
#define CYCLONE_V_MEM_BASE_ADDR  0x20000000//512M
#define CYCLONE_V_MEM_SIZE  0x20000000//512M

#define DEVNAME "Cyclone_V_dev"

void *regs_mem;
void *data_mem;

static DEFINE_SEMAPHORE(interrupt_mutex);
static DECLARE_WAIT_QUEUE_HEAD(interrupt_wq);

static int interrupt_flag = 0;//, curdev = 0, psize=0, cmdid=0;
wait_queue_head_t intr_wait;

static DEFINE_SPINLOCK(interrupt_flag_lock);

static dev_t first; // Global variable for the first device number 
static struct cdev c_dev; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class

#define MAP_SIZE           (4096)
#define MAP_MASK           (MAP_SIZE-1)

off_t byte_addr=0xFFC25080;

static irq_handler_t __test_isr0(int irq, void *dev_id, struct pt_regs *regs)
{
    printk(KERN_INFO DEVNAME ": ISR0\n");
	spin_lock(&interrupt_flag_lock);
    interrupt_flag = 1;
	spin_unlock(&interrupt_flag_lock);
    wake_up_interruptible(&interrupt_wq);
    return (irq_handler_t)IRQ_HANDLED;
}


/*static irq_handler_t __test_isr1(int irq, void *dev_id, struct pt_regs *regs)
{
    printk(KERN_INFO DEVNAME ": ISR1\n");
	spin_lock(&interrupt_flag_lock);
    interrupt_flag = 2;
	spin_unlock(&interrupt_flag_lock);
	return (irq_handler_t)IRQ_HANDLED;
}*/

uint32_t cyclone_V_read_reg(uint32_t offset)
{
  uint32_t value;
  value=ioread32((void *)(regs_mem + offset*4));
//  value=readl(adcdevice->bar0 + offset);
  printk(KERN_INFO "REG read: 0x%08X = 0x%08X\n", offset, value);
  return(value);
}

void cyclone_V_write_reg(uint32_t offset, uint32_t value)
{
  iowrite32(value, regs_mem + offset*4);
//  writel(value, adcdevice->bar0 + offset);
  printk(KERN_INFO "REG write: 0x%08X = 0x%08X\n", offset, value);
}

void irq_enable(void)
{
  uint32_t value;
	value=cyclone_V_read_reg(12); //!!! need fix !!!
	cyclone_V_write_reg(12, (1<<25)|value);//!!! need fix !!!
}

int cyclone_V_blockread(uint8_t npage, uint32_t *data)
{
  int pagesize = sizeof(uint32_t) * MEM_PAGE_SEZE;
  if (copy_to_user((void *)data, (void *)(data_mem + npage * pagesize), pagesize)) return -EFAULT;
  return 0;
}

int cyclone_V_nblockread(uint8_t npage, uint8_t pages, uint32_t *data)
{
//  for(i = 0; i < pages; i++)
  if (copy_to_user((void *)data, (void *)(data_mem + npage * MEM_PAGE_SEZE_IN_BYTES), pages*MEM_PAGE_SEZE_IN_BYTES)) return -EFAULT;
  return 0;
}

int cyclone_V_blockwrite(uint8_t npage, uint32_t *data)
{
  if (copy_from_user((void *)(data_mem + npage * MEM_PAGE_SEZE_IN_BYTES), (void *)data, MEM_PAGE_SEZE_IN_BYTES)) return -EFAULT;
  return 0;
}

int cyclone_V_nblockwrite(uint8_t npage, uint8_t pages, uint32_t *data)
{
  if (copy_from_user((void *)(data_mem + npage * MEM_PAGE_SEZE_IN_BYTES), (void *)data, pages*MEM_PAGE_SEZE_IN_BYTES)) return -EFAULT;
//тут пока пишем только одну страницу, все остальные копия
//   for(i = 0; i < pages; i++)
//   if (copy_from_user((void *)(data_mem + (npage+i) * pagesize), (void *)&data[i*1024], pagesize)) return -EFAULT;
  return 0;
}

static int cyclone_V_open(struct inode *i, struct file *f)
{
  printk(KERN_INFO DEVNAME ": open()\n");
  return 0;
}

static int cyclone_V_close(struct inode *i, struct file *f)
{
  printk(KERN_INFO DEVNAME ": close()\n");
  return 0;
}

static ssize_t cyclone_V_read(struct file *f, char __user *buf, size_t
  len, loff_t *off)
{
	int ret=0;

//	if (down_trylock(&interrupt_mutex))
//		return -EAGAIN;
//do
//{
//	if (wait_event_interruptible(interrupt_wq, interrupt_flag != 0)) {
//    ret=wait_event_interruptible_timeout(interrupt_wq, interrupt_flag != 0, 500);
	if (wait_event_interruptible(interrupt_wq, interrupt_flag != 0)) {
		ret = -ERESTART;
		goto release_and_exit;
	}

/*     curdev=ioread32(table_mem+(0<<2));
     cmdid=ioread32(table_mem+(1<<2))&0xFF;
     psize=(curdev>>8)&0xFF;
     curdev=curdev&0xFF;
*/
//} while (!((cmdid==0x55)&&(curdev==1)));

//	spin_lock(&interrupt_flag_lock);
//	interrupt_flag = 0;
//	spin_unlock(&interrupt_flag_lock);

release_and_exit:
//	up(&interrupt_mutex);

	return ret;
}

static ssize_t cyclone_V_write(struct file *f, const char __user *buf,
  size_t len, loff_t *off)
{
  printk(KERN_INFO DEVNAME ": write()\n");
  return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
int cyclone_V_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#else
static long cyclone_V_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	cyclone_v_reg_t devReg;
    cyclone_v_recvblock_t devRecvBlock;
//    uint32_t value32;
//    uint16_t value16;

	if(_IOC_TYPE(cmd) != CYCLONE_V_IOC_MAGIC){
		printk(KERN_INFO "cyclone_v: ioctl: unknown command:%08x, %X, %c\n", cmd, _IOC_TYPE(cmd), _IOC_TYPE(cmd));
		return -EINVAL;
	}
	if(_IOC_NR(cmd) > CYCLONE_V_IOC_MAXNR){
		printk(KERN_INFO "cyclone_v: ioctl: command nr too high: %08x\n", cmd);
		return -EINVAL;
	}

        switch(cmd)
        {
          case CYCLONE_V_IOC_REG_READ:
//                  memset(&devReg, 0, sizeof(devReg));
                  if (copy_from_user((void *) &devReg, (void *)arg, sizeof(devReg))) return -EFAULT;
                  devReg.value=cyclone_V_read_reg(devReg.offset);
                  if (copy_to_user((void *)arg, &devReg, sizeof(devReg))) return -EFAULT;
                  break;
          case CYCLONE_V_IOC_REG_WRITE:
//                  memset(&devReg, 0, sizeof(devReg));
                  if (copy_from_user((void *) &devReg, (void *)arg, sizeof(devReg))) return -EFAULT;
                  cyclone_V_write_reg(devReg.offset, devReg.value);
                  break;
          case CYCLONE_V_IOC_BLOCKREAD:
                  if (copy_from_user((void *) &devRecvBlock, (void *)arg, sizeof(devRecvBlock))) return -EFAULT;
                  cyclone_V_nblockread(devRecvBlock.npage, devRecvBlock.pages, devRecvBlock.data);
                  break;
          case CYCLONE_V_IOC_BLOCKWRITE:
                  if (copy_from_user((void *) &devRecvBlock, (void *)arg, sizeof(devRecvBlock))) return -EFAULT;
                  cyclone_V_nblockwrite(devRecvBlock.npage, devRecvBlock.pages, devRecvBlock.data);
                  break;
          case CYCLONE_V_IOC_WAIT_INT1:
//                  adcdevice->intr_flag = 0;
//                  value=wait_event_interruptible_timeout(adcdevice->intr_wait, adcdevice->intr_flag != 0, 20);
//                  printk("value=%u\n", value);
//                  if (copy_to_user((void *)arg, (void *)&value, sizeof(value))) return -EFAULT;
                  break;
        }
 return 0;
}

static struct file_operations pugs_fops =
{
  .owner = THIS_MODULE,
  .open = cyclone_V_open,
  .release = cyclone_V_close,
  .read = cyclone_V_read,
  .write = cyclone_V_write,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
  .ioctl = cyclone_V_ioctl
#else
  .unlocked_ioctl = cyclone_V_ioctl
#endif
};

static int cyclone_V_driver_probe(struct platform_device *pdev)
{
    int ret;
    void *map_page_addr; 

	int irq0_num;//, irq1_num;

	if (alloc_chrdev_region(&first, 0, 1, "Shweta") < 0)
	{
		ret = -EFAULT;
		goto fail_alloc_chardev;
	}
	printk(KERN_INFO DEVNAME ": alloc_chrdev_region is OK\n");


    if ((cl = class_create(THIS_MODULE, "chardrv")) == NULL)
	{
		ret = -EFAULT;
		goto fail_class_create;
	}
	printk(KERN_INFO DEVNAME ": class_create is OK\n");

	if (device_create(cl, NULL, first, NULL, DEVNAME) == NULL)
	{
		ret = -EFAULT;
		goto fail_device_create;
	}
	printk(KERN_INFO DEVNAME ": device_create is OK\n");

	cdev_init(&c_dev, &pugs_fops);
	if (cdev_add(&c_dev, first, 1) == -1)
	{
		ret = -EFAULT;
		goto fail_cdev_add;
	}

	if (!request_mem_region(byte_addr& ~MAP_MASK, PAGE_SIZE, DEVNAME))
	{
	        printk(KERN_INFO DEVNAME ": request_mem_region error\n");
		ret = -EBUSY;
		goto fail_request_mem_cpureg;
	}

	map_page_addr = ioremap(byte_addr& ~MAP_MASK, PAGE_SIZE);
	if (map_page_addr == NULL) {
		ret = -EFAULT;
        printk(KERN_INFO DEVNAME ": error SDRAM initialization\n");
		goto fail_ioremap_mem_cpureg;
	}

    printk(KERN_INFO DEVNAME ": map_byte_addr ioremap is OK\n");
    iowrite32(0x3fff, map_page_addr + (byte_addr & MAP_MASK)); //initialization SDRAM
    printk(KERN_INFO DEVNAME ": init SDRAM successed\n");

	iounmap(map_page_addr);

fail_ioremap_mem_cpureg:
 	release_mem_region(byte_addr& ~MAP_MASK, PAGE_SIZE);
/////////////////////////////////////////////////////////////

	if (!request_mem_region(CYCLONE_V_MEM_BASE_ADDR, CYCLONE_V_MEM_SIZE, DEVNAME))
	{
     	printk(KERN_INFO DEVNAME ": request_mem_region memory Error\n");
		ret = -EBUSY;
		goto fail_request_mem;
	}

	data_mem = ioremap(CYCLONE_V_MEM_BASE_ADDR, CYCLONE_V_MEM_SIZE);
	if (data_mem == NULL) {
     	printk(KERN_INFO DEVNAME ": ioremap memory Error\n");
		ret = -EFAULT;
		goto fail_ioremap_mem;
	}
	printk(KERN_INFO DEVNAME ": ioremap memory is OK\n");

	if (!request_mem_region(CYCLONE_V_REGS_BASE_ADDR, CYCLONE_V_REGS_SIZE, DEVNAME))
	{
     	printk(KERN_INFO DEVNAME ": request_mem_region regs Error\n");
		ret = -EBUSY;
		goto fail_request_reg;
	}

	regs_mem = ioremap(CYCLONE_V_REGS_BASE_ADDR, CYCLONE_V_REGS_SIZE);
	if (regs_mem == NULL) {
     	printk(KERN_INFO DEVNAME ": ioremap regs Error\n");
		ret = -EFAULT;
		goto fail_ioremap_reg;
	}
	printk(KERN_INFO DEVNAME ": ioremap regs is OK\n");

//    iowrite32(0x8000, table_mem+0x00000100);
//	printk(KERN_INFO DEVNAME ": send clear command\n");

	/* IRQ0 */
	irq0_num = platform_get_irq_byname(pdev, "IRQ0");
	if (irq0_num == -ENXIO) {
		printk(KERN_INFO DEVNAME ": cannot obtain IRQ0\n");
		ret = -ENXIO;
		goto fail_platform_get_irq_byname;
	}
	printk(KERN_INFO DEVNAME ": IRQ %d about to be registered!\n",irq0_num);

	/* IRQ1 */
/*	irq1_num = platform_get_irq_byname(pdev, "IRQ1");
	if (irq1_num == -ENXIO) {
		printk(KERN_INFO DEVNAME ": cannot obtain IRQ1\n");
		ret = -ENXIO;
		goto fail_platform_get_irq_byname;
	}
	printk(KERN_INFO DEVNAME ": IRQ %d about to be registered!\n",irq1_num);
*/

	ret = request_irq(irq0_num, (irq_handler_t)__test_isr0, 0, DEVNAME, NULL);
	if (ret) {
		ret = -EFAULT;
		printk(KERN_INFO DEVNAME ": Unable to register IRQ0 interrupt %d\n", irq0_num);
		goto fail_request_irq0_error;
	}
/*
	ret = request_irq(irq1_num, (irq_handler_t)__test_isr1, 0, DEVNAME, NULL);
	if (ret) {
		ret = -EFAULT;
		printk(KERN_INFO DEVNAME ": Unable to register IRQ1 interrupt %d\n", irq1_num);
		goto fail_request_irq1_error;
	}
*/

//    cyclone_V_write_reg(CYCLONE_V_REG_CSR, value32); //разрешение работы по прерываниям

//    init_waitqueue_head(&intr_wait);
    irq_enable();

	return 0;

/*fail_request_irq1_error:
	free_irq(irq0_num, pdev);
*/

fail_request_irq0_error:

fail_platform_get_irq_byname:
	iounmap(regs_mem);

fail_ioremap_reg:
	release_mem_region(CYCLONE_V_REGS_BASE_ADDR, CYCLONE_V_REGS_SIZE);

fail_request_reg:
	iounmap(data_mem);

fail_ioremap_mem:
	release_mem_region(CYCLONE_V_MEM_BASE_ADDR, CYCLONE_V_MEM_SIZE);

fail_request_mem:

fail_request_mem_cpureg:
	cdev_del(&c_dev);

fail_cdev_add:
    device_destroy(cl, first);

fail_device_create:
    class_destroy(cl);

fail_class_create:
    unregister_chrdev_region(first, 1);

fail_alloc_chardev:
	return ret;
}

static int cyclone_V_driver_remove(struct platform_device *pdev)
{
    int ret;
	int irq0_num;//, irq1_num;

	/* IRQ0 */
	irq0_num = platform_get_irq_byname(pdev, "IRQ0");
	if (irq0_num == -ENXIO) {
		printk(KERN_INFO DEVNAME ": cannot obtain IRQ0\n");
		ret = -ENXIO;
		goto error_deinit;//next_deinit;
	}
	printk(KERN_INFO DEVNAME ": IRQ %d about to be unregistered!\n",irq0_num);
	free_irq(irq0_num, NULL);

//next_deinit:

/*	irq1_num = platform_get_irq_byname(pdev, "IRQ1");
	if (irq1_num == -ENXIO) {
		printk(KERN_INFO DEVNAME ": cannot obtain IRQ1\n");
		ret = -ENXIO;
		goto error_deinit;
	}
	printk(KERN_INFO DEVNAME ": IRQ %d about to be unregistered!\n",irq1_num);
	free_irq(irq1_num, NULL);
*/

	iounmap(regs_mem);
	release_mem_region(CYCLONE_V_REGS_BASE_ADDR, CYCLONE_V_REGS_SIZE);

	iounmap(data_mem);
	release_mem_region(CYCLONE_V_MEM_BASE_ADDR, CYCLONE_V_MEM_SIZE);

	cdev_del(&c_dev);
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);
	printk(KERN_INFO DEVNAME ": char dev unregistered");
	return 0;
error_deinit:
	return ret;
}

static const struct of_device_id cyclone_V_driver_id[]={
{
	.compatible="altr,socfpga-mysoftip"},
{}
};

static struct platform_driver cyclone_V_driver={
	.driver={
	.name=DEVNAME,
	.owner=THIS_MODULE,
	.of_match_table=of_match_ptr(cyclone_V_driver_id),
	},
	.probe=cyclone_V_driver_probe,
	.remove=cyclone_V_driver_remove
};

module_platform_driver(cyclone_V_driver);

MODULE_LICENSE("GPL");
