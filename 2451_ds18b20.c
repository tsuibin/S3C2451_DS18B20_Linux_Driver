#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include <linux/gpio.h>
#include <plat/gpio-cfg.h>

#define DEVICE_NAME "TEM"
#define tp_MAJOR 232


//static unsigned char data[2];
void tm_18b20_reset(void) //18b20初始化
{   
	s3c_gpio_cfgpin(S3C2451_GPN(8), S3C_GPIO_SFN(1));//设置引脚为输出
	gpio_set_value(S3C2451_GPN(8), 1);
	udelay(5);
	gpio_set_value(S3C2451_GPN(8), 0);//产生下降沿
	udelay(600);//持续600us低电平
	gpio_set_value(S3C2451_GPN(8), 1);//拉回高电平
	udelay(60);//持续60us
	s3c_gpio_cfgpin(S3C2451_GPN(8), S3C_GPIO_SFN(0));//设置引脚为输入
}  

void tm_18b20_writeb(unsigned char dat) //写一个字节函数
{   
	unsigned char j;
	s3c_gpio_cfgpin(S3C2451_GPN(8), S3C_GPIO_SFN(1));//输出模式
	
	for (j = 1; j <= 8; j++){
		gpio_set_value(S3C2451_GPN(8), 0);//产生下降延
		udelay(1);
		if((dat&0x01)==1)//根据dat的位值来设置数据线值
		gpio_set_value(S3C2451_GPN(8), 1);
		
		udelay(60);
		gpio_set_value(S3C2451_GPN(8), 1);//拉回高电平
		udelay(10);
		dat = dat >> 1;
	}
	
	gpio_set_value(S3C2451_GPN(8), 1);//设置完后将端口拉回高电平
}


unsigned char tm_18b20_readb(void) //读一个字节函数
{   
	unsigned char i,temp=0;   
	
	for (i = 1; i <= 8; i++){
		s3c_gpio_cfgpin(S3C2451_GPN(8), S3C_GPIO_SFN(1));//设置端口为输出
		gpio_set_value(S3C2451_GPN(8), 0);//产生下降延
		udelay(1);
		temp >>= 1;
		gpio_set_value(S3C2451_GPN(8), 1);//拉回高电平
		s3c_gpio_cfgpin(S3C2451_GPN(8), S3C_GPIO_SFN(0));//设置端口为输入
		udelay(10);
		if( gpio_get_value(S3C2451_GPN(8))) //如果为高电平
		temp = temp | 0x80;
		udelay(60);  
	}  
	return (temp);   
}

void DS18B20PRO(void)   
{   
	tm_18b20_reset(); //复位
	udelay(420);
	tm_18b20_writeb(0xcc); //跳过序列号命令   
	tm_18b20_writeb(0x44); //发转换命令 44H,
	
	mdelay(750);
	
	tm_18b20_reset (); //复位
	udelay(400);
	tm_18b20_writeb(0xcc); //跳过序列号命令   
	tm_18b20_writeb(0xbe); //发送读取命令
	//data[0] = tm_18b20_readb(); //读取低位温度   
	//data[1] = tm_18b20_readb(); //读取高位温度   
}  

static ssize_t s3c2451_18b20_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
	DS18B20PRO();
	
	buf[0] = tm_18b20_readb();
	buf[1] = tm_18b20_readb();
//	printk("%x,%x\n",buf[0],buf[1]);
	return 0;
}

static struct file_operations s3c2451_18b20_fops = {
	.owner = THIS_MODULE,
	.read = s3c2451_18b20_read,
};

static struct cdev cdev_18b20;

static int __init s3c2451_18b20_init(void)
{
	int result;
	dev_t devno = MKDEV(tp_MAJOR,0);
	struct class *tem_class;
	
	result = register_chrdev_region(devno,1,DEVICE_NAME);
	
	if(result){
		printk(KERN_NOTICE "Error %d register 18b20",result);
		return result;
	}
	
	cdev_init(&cdev_18b20,&s3c2451_18b20_fops);
	
	result = cdev_add(&cdev_18b20,devno,1);
	if(result){
		printk(KERN_NOTICE "Error %d adding 18b20",result);
		return result;
	}
	
	tem_class = class_create(THIS_MODULE, "tem_class");
	device_create(tem_class, NULL, MKDEV(tp_MAJOR, 0), "ds18b20","DS18dev%d", 0);
	
	return 0;  
}

static void __exit s3c2451_18b20_exit(void)
{
	cdev_del(&cdev_18b20);
	unregister_chrdev_region(MKDEV(tp_MAJOR,0),1);
}


module_init(s3c2451_18b20_init);
module_exit(s3c2451_18b20_exit);

MODULE_AUTHOR("CaZool");
MODULE_LICENSE("GPL");


