#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/device.h> /* device_create()*/
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <asm/atomic.h>
#include <asm/unistd.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/clk.h>
#include <linux/wait.h>


#define MEMDEV_MAJOR 251

#define MEMDEV_NR_DEVS 2


struct mem_dev                                     
{                                                        
  char *data;                      
  unsigned long size;       
};

static int mem_major = MEMDEV_MAJOR;

module_param(mem_major, int, S_IRUGO);

struct mem_dev *mem_devp;



#define DS18B20_PIN   S3C2451_GPF(0)
#define DS18B20_PIN_OUTP S3C2451_GPIO_OUTPUT
#define DS18B20_PIN_INP   S3C2451_GPIO_INPUT
#define HIGH 1
#define LOW 0

// DS18B20复位函数
unsigned char DS18b20_reset (void)
{
    // 配置GPIOB0输出模式
    s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_OUTP);
    
    // 向18B20发送一个上升沿，并保持高电平状态约100微秒
    s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
    udelay(100);
    
    // 向18B20发送一个下降沿，并保持低电平状态约600微秒
    s3c2451_gpio_setpin(DS18B20_PIN, LOW);
    udelay(600);
    
    // 向18B20发送一个上升沿，此时可释放DS18B20总线
    s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
    udelay(100);
    
    // 以上动作是给DS18B20一个复位脉冲
    // 通过再次配置GPIOB1引脚成输入状态，可以检测到DS18B20是否复位成功
    s3c2410_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_INP);
    
    // 若总线在释放后总线状态为高电平，则复位失败
    if(s3c2451_gpio_getpin(DS18B20_PIN))
	{ printk("DS18b20 reset failed.\r\n"); return 1;}

    return 0;
} 


void DS18b20_write_byte (unsigned char  byte)
{
    char i;
    // 配置GPIOB1为输出模式
    s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_OUTP);

    // 写“1”时隙：
    //     保持总线在低电平1微秒到15微秒之间
    //     然后再保持总线在高电平15微秒到60微秒之间
    //     理想状态: 1微秒的低电平然后跳变再保持60微秒的高电平
    //
    // 写“0”时隙：
    //     保持总线在低电平15微秒到60微秒之间
    //     然后再保持总线在高电平1微秒到15微秒之间
    //     理想状态: 60微秒的低电平然后跳变再保持1微秒的高电平
    for (i = 0; i < 8; i++)
    {
        s3c2451_gpio_setpin(DS18B20_PIN, LOW); udelay(1);
        if(byte & HIGH)
        {
             // 若byte变量的D0位是1，则需向总线上写“1”
             // 根据写“1”时隙规则，电平在此处翻转为高
             s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
        }
        else 
        {
             // 若byte变量的D0位是0，则需向总线上写“0”
             // 根据写“0”时隙规则，电平在保持为低
             // s3c2451_gpio_setpin(DS18B20_PIN, LOW);
        }
        // 电平状态保持60微秒
        udelay(60);

        s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
        udelay(15);

        byte >>= 1;
    }
    s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
} 

unsigned char DS18b20_read_byte (void)
{
  char i = 0;
   unsigned char byte = 0;
    // 读“1”时隙：
    //     若总线状态保持在低电平状态1微秒到15微秒之间
    //     然后跳变到高电平状态且保持在15微秒到60微秒之间
    //      就认为从DS18B20读到一个“1”信号
    //     理想情况: 1微秒的低电平然后跳变再保持60微秒的高电平
    //
    // 读“0”时隙：
    //     若总线状态保持在低电平状态15微秒到30微秒之间
    //     然后跳变到高电平状态且保持在15微秒到60微秒之间
    //     就认为从DS18B20读到一个“0”信号
    //     理想情况: 15微秒的低电平然后跳变再保持46微秒的高电平
    for (i = 0; i < 8; i++)
    {
        s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_OUTP); 
        s3c2451_gpio_setpin(DS18B20_PIN, LOW);

        udelay(1);
        byte >>= 1;

        s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
        s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_INP);

        // 若总线在我们设它为低电平之后若1微秒之内变为高
        // 则认为从DS18B20处收到一个“1”信号
        // 因此把byte的D7为置“1”

        if (s3c2451_gpio_getpin(DS18B20_PIN)) byte |= 0x80;
        udelay(60);
    }
    return byte;       
}

char  DS18b20_proc(int *major ,int *minor)         
{
	int temp=0;
	int i=0;

    if(DS18b20_reset())
	goto end;
	udelay(120);
	DS18b20_write_byte(0xcc);
    DS18b20_write_byte(0x44);
	udelay(5);
	if(DS18b20_reset())
	goto end;
    udelay(200);
	DS18b20_write_byte(0xcc);
    DS18b20_write_byte(0xbe);
	int b= DS18b20_read_byte();
	int a=DS18b20_read_byte();
	a<<=8;
	a|=b;
	a&=0x7ff;
	if(a&1)
	 temp+=6;
	 a>>=1;
	 if(a&1)
	 temp+=12;
	 a>>=1;
	 if(a&1)
	 temp+=25;
	 a>>=1;
	 if(a&1)
	 temp+=50;
	 a>>=1;
	 
	 
	 *minor=temp;
	 *major=a;
end :
		
	return 0;
} 



/*文件打开函数*/
int mem_open(struct inode *inode, struct file *filp)
{

       return 0;
}


/*文件释放函数*/
int mem_release(struct inode *inode, struct file *filp)
{
  return 0;
}

/*读函数*/
int x=0;
static ssize_t mem_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	  int major,minor;
	  DS18b20_proc(&major,&minor);
	  char data[2];
	  data[0]=major;
	  data[1]=minor;
	  
	  copy_to_user(buf, data, 2);
	  
	  return 0;
  

}

/*写函数*/
static ssize_t mem_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
  printk("\nwrite \n");
 // GPBDAT=0;

  return size;
}

/* seek文件定位函数 */


/*文件操作结构体*/
static const struct file_operations mem_fops =
{
  .owner = THIS_MODULE,
  .read = mem_read,
  .write = mem_write,
  .open = mem_open,
  .release = mem_release,
};
struct cdev cdev; 


/*设备驱动模块加载函数*/
static int memdev_init(void)
{
	struct class *myclass;
	int result;
	dev_t devno;
	result = alloc_chrdev_region(&devno, 0, 1, "memdev");
	mem_major = MAJOR(devno);
 
  if (result < 0)
    return result;
	printk("\nmy DS18B20 \n");

  /*初始化cdev结构*/
  cdev_init(&cdev, &mem_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &mem_fops;
  
  /* 注册字符设备 */
  cdev_add(&cdev, devno, 1);
   
  /* 为设备描述结构分配内存*/
  mem_devp = kmalloc( sizeof(struct mem_dev), GFP_KERNEL);
  if (!mem_devp)    /*申请失败*/
  {
    result =  - ENOMEM;
    goto fail_malloc;
  }
  
  memset(mem_devp, 0, sizeof(struct mem_dev));
  
 
 
  /*自动创建设备文件*/
	myclass = class_create(THIS_MODULE,"test_char"); /*在sys下创建类目录/sys/class/test_char*/
	device_create(myclass, NULL, MKDEV(mem_major,0), NULL, "ds18b20");   
	printk("\nDS18B20 inited\n");
  return 0;

  fail_malloc: 
  unregister_chrdev_region(devno, 1);
  
  return result;
}

/*模块卸载函数*/
static void memdev_exit(void)
{
  cdev_del(&cdev);   /*注销设备*/
  kfree(mem_devp);     /*释放设备结构体内存*/
  unregister_chrdev_region(MKDEV(mem_major, 0), 2); /*释放设备号*/
  printk("\n DS18B20 EXIT\n");
  
}

MODULE_AUTHOR("CaZool");
MODULE_LICENSE("GPL");

module_init(memdev_init);
module_exit(memdev_exit);

