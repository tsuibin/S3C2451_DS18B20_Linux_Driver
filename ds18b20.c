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

// DS18B20��λ����
unsigned char DS18b20_reset (void)
{
    // ����GPIOB0���ģʽ
    s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_OUTP);
    
    // ��18B20����һ�������أ������ָߵ�ƽ״̬Լ100΢��
    s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
    udelay(100);
    
    // ��18B20����һ���½��أ������ֵ͵�ƽ״̬Լ600΢��
    s3c2451_gpio_setpin(DS18B20_PIN, LOW);
    udelay(600);
    
    // ��18B20����һ�������أ���ʱ���ͷ�DS18B20����
    s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
    udelay(100);
    
    // ���϶����Ǹ�DS18B20һ����λ����
    // ͨ���ٴ�����GPIOB1���ų�����״̬�����Լ�⵽DS18B20�Ƿ�λ�ɹ�
    s3c2410_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_INP);
    
    // ���������ͷź�����״̬Ϊ�ߵ�ƽ����λʧ��
    if(s3c2451_gpio_getpin(DS18B20_PIN))
	{ printk("DS18b20 reset failed.\r\n"); return 1;}

    return 0;
} 


void DS18b20_write_byte (unsigned char  byte)
{
    char i;
    // ����GPIOB1Ϊ���ģʽ
    s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_OUTP);

    // д��1��ʱ϶��
    //     ���������ڵ͵�ƽ1΢�뵽15΢��֮��
    //     Ȼ���ٱ��������ڸߵ�ƽ15΢�뵽60΢��֮��
    //     ����״̬: 1΢��ĵ͵�ƽȻ�������ٱ���60΢��ĸߵ�ƽ
    //
    // д��0��ʱ϶��
    //     ���������ڵ͵�ƽ15΢�뵽60΢��֮��
    //     Ȼ���ٱ��������ڸߵ�ƽ1΢�뵽15΢��֮��
    //     ����״̬: 60΢��ĵ͵�ƽȻ�������ٱ���1΢��ĸߵ�ƽ
    for (i = 0; i < 8; i++)
    {
        s3c2451_gpio_setpin(DS18B20_PIN, LOW); udelay(1);
        if(byte & HIGH)
        {
             // ��byte������D0λ��1��������������д��1��
             // ����д��1��ʱ϶���򣬵�ƽ�ڴ˴���תΪ��
             s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
        }
        else 
        {
             // ��byte������D0λ��0��������������д��0��
             // ����д��0��ʱ϶���򣬵�ƽ�ڱ���Ϊ��
             // s3c2451_gpio_setpin(DS18B20_PIN, LOW);
        }
        // ��ƽ״̬����60΢��
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
    // ����1��ʱ϶��
    //     ������״̬�����ڵ͵�ƽ״̬1΢�뵽15΢��֮��
    //     Ȼ�����䵽�ߵ�ƽ״̬�ұ�����15΢�뵽60΢��֮��
    //      ����Ϊ��DS18B20����һ����1���ź�
    //     �������: 1΢��ĵ͵�ƽȻ�������ٱ���60΢��ĸߵ�ƽ
    //
    // ����0��ʱ϶��
    //     ������״̬�����ڵ͵�ƽ״̬15΢�뵽30΢��֮��
    //     Ȼ�����䵽�ߵ�ƽ״̬�ұ�����15΢�뵽60΢��֮��
    //     ����Ϊ��DS18B20����һ����0���ź�
    //     �������: 15΢��ĵ͵�ƽȻ�������ٱ���46΢��ĸߵ�ƽ
    for (i = 0; i < 8; i++)
    {
        s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_OUTP); 
        s3c2451_gpio_setpin(DS18B20_PIN, LOW);

        udelay(1);
        byte >>= 1;

        s3c2451_gpio_setpin(DS18B20_PIN, HIGH);
        s3c2451_gpio_cfgpin(DS18B20_PIN, DS18B20_PIN_INP);

        // ����������������Ϊ�͵�ƽ֮����1΢��֮�ڱ�Ϊ��
        // ����Ϊ��DS18B20���յ�һ����1���ź�
        // ��˰�byte��D7Ϊ�á�1��

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



/*�ļ��򿪺���*/
int mem_open(struct inode *inode, struct file *filp)
{

       return 0;
}


/*�ļ��ͷź���*/
int mem_release(struct inode *inode, struct file *filp)
{
  return 0;
}

/*������*/
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

/*д����*/
static ssize_t mem_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
  printk("\nwrite \n");
 // GPBDAT=0;

  return size;
}

/* seek�ļ���λ���� */


/*�ļ������ṹ��*/
static const struct file_operations mem_fops =
{
  .owner = THIS_MODULE,
  .read = mem_read,
  .write = mem_write,
  .open = mem_open,
  .release = mem_release,
};
struct cdev cdev; 


/*�豸����ģ����غ���*/
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

  /*��ʼ��cdev�ṹ*/
  cdev_init(&cdev, &mem_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &mem_fops;
  
  /* ע���ַ��豸 */
  cdev_add(&cdev, devno, 1);
   
  /* Ϊ�豸�����ṹ�����ڴ�*/
  mem_devp = kmalloc( sizeof(struct mem_dev), GFP_KERNEL);
  if (!mem_devp)    /*����ʧ��*/
  {
    result =  - ENOMEM;
    goto fail_malloc;
  }
  
  memset(mem_devp, 0, sizeof(struct mem_dev));
  
 
 
  /*�Զ������豸�ļ�*/
	myclass = class_create(THIS_MODULE,"test_char"); /*��sys�´�����Ŀ¼/sys/class/test_char*/
	device_create(myclass, NULL, MKDEV(mem_major,0), NULL, "ds18b20");   
	printk("\nDS18B20 inited\n");
  return 0;

  fail_malloc: 
  unregister_chrdev_region(devno, 1);
  
  return result;
}

/*ģ��ж�غ���*/
static void memdev_exit(void)
{
  cdev_del(&cdev);   /*ע���豸*/
  kfree(mem_devp);     /*�ͷ��豸�ṹ���ڴ�*/
  unregister_chrdev_region(MKDEV(mem_major, 0), 2); /*�ͷ��豸��*/
  printk("\n DS18B20 EXIT\n");
  
}

MODULE_AUTHOR("CaZool");
MODULE_LICENSE("GPL");

module_init(memdev_init);
module_exit(memdev_exit);
