#include <linux/module.h> /* 模块API相关的头文件 */
#include <linux/pci.h> /* 内核打印 printk 的头文件 */
#include <linux/fs.h> /* static struct file_operations 的头文件 */
#include <linux/cdev.h>//cdev头文件
#include <linux/uaccess.h>//处理用户空间和内核空间数据传递接口的头文件
#include <linux/string.h>//额，linux不能使用普通的string.h,需要使用内核的string.h
#include <asm/io.h>
#include <asm/uaccess.h>

#include "leddev_rk3568.h"
/////////////////////////////////////////////////

/////////////////////////////////////////////////


#define DEVICE_NAME			"myleddev"
#define CLASS_NAME			"myleddev_class"

#define BUFFER_SIZE			100

static dev_t dev_num = 0;
static struct cdev my_cdev;
static struct class* my_class = NULL;
static struct device *my_device = NULL;

static char kernel_buf[10] = "off";
static char write_buf[BUFFER_SIZE] = {0};
static char read_buf[BUFFER_SIZE] = {0};


static void __iomem* V_GPIOC_0_MUX = NULL;
static void __iomem* V_GPIOC_0_DS = NULL;
static void __iomem* V_GPIOC_0_DDR = NULL;
static void __iomem* V_GPIOC_0_DR = NULL;


int led_on(void)
{
	u32 val = 0;
	val = readl(V_GPIOC_0_DR);
	val &= ~(1<<0);
	val |= (1<<16);
	writel(val, V_GPIOC_0_DR);
	printk("led on ,val:0x%x\r\n", val);
	return 0;
}


void led_remap(void)
{
	V_GPIOC_0_MUX = ioremap(GPIOC_0_MUX, 4);
	V_GPIOC_0_DS = ioremap(GPIOC_0_DS, 4);
	V_GPIOC_0_DDR = ioremap(GPIOC_0_DDR, 4);
	V_GPIOC_0_DR = ioremap(GPIOC_0_DR, 4);
}

void led_unmap(void)
{
	iounmap(V_GPIOC_0_MUX);
	iounmap(V_GPIOC_0_DS);
	iounmap(V_GPIOC_0_DDR);
	iounmap(V_GPIOC_0_DR);
}

int led_off(void)
{
	u32 val = 0;
	val = readl(V_GPIOC_0_DR);
	val |= (1<<16);
	val |= 1<<0;
	writel(val, V_GPIOC_0_DR);
	printk("led off ,0x%x\r\n", val);
	return 0;
}

int myleddev_open(struct inode *inode, struct file *file)
{
	u32 val = 0;
	printk("myleddev_open\r\n");

	//初始化GPIO
	// 0.地址映射
	led_remap();

	// 1.IO复用
	val = readl(V_GPIOC_0_MUX);
	val &= ~(0x7<<16);
	val |= (0x7<<16);
	writel(val, V_GPIOC_0_MUX);

	// 2.配置驱动能力
	val = readl(V_GPIOC_0_DS);
	val &= ~(0x3f<<0);
	val |= (0x3f<<0);
	val |= (0x3f<<16);//使能写
	writel(val, V_GPIOC_0_DS);

	// 3.配置输入输出方向
	val &= ~(0x1<<0);//貌似是因为这里写错了才导致灯不亮的，有机会试试
	val |= (0x1<<0);
	val |= (0x1<<16);
	writel(val, V_GPIOC_0_DDR);

	// 4.配置默认高低电平
	val |= (0x1<<0);
	val |= (0x1<<16);
	writel(val, V_GPIOC_0_DR);

	return 0;
}


int myleddev_release(struct inode *inode, struct file *file)
{
	printk("myleddev_release\r\n");
	led_unmap();
	return 0;
}

ssize_t myleddev_read(struct file *file, char __user *buf, size_t len, loff_t * offset)
{
	int bytes_read;
	printk("myleddev_read\r\n");

    if (*offset >= BUFFER_SIZE){
		return 0;
	}
	
	// 拷贝内核数据到读缓冲区
	memcpy(read_buf, kernel_buf, sizeof(kernel_buf));

	// 将数据拷贝到用户buf
    bytes_read = copy_to_user(buf, read_buf, len);
    *offset += bytes_read;

    printk(KERN_INFO "mymyleddev: Read %d bytes\n", bytes_read);
    return bytes_read;
}

ssize_t myleddev_write(struct file *file, const char __user *buf, size_t len, loff_t * offset)
{
	int bytes_written;
	printk("myleddev_write\r\n");
    if (*offset >= BUFFER_SIZE){
		return -ENOSPC;
	}

	memset(write_buf, 0, BUFFER_SIZE);

    bytes_written = copy_from_user(write_buf, buf, len);
    *offset += bytes_written;
	printk("write:%s, len=%lu\r\n", write_buf, len);

	if(strcmp(write_buf, "on") == 0)
	{
		//on
		led_on();
	}
	if(strcmp(write_buf, "off") == 0)
	{
		//off
		led_off();
	}
	memset(kernel_buf, 0, 10);
	memcpy(kernel_buf, write_buf, strlen(write_buf));

    printk(KERN_INFO "mymyleddev: Wrote %d bytes\r\n", bytes_written);
    return bytes_written;
}


// 创建一个文件操作结构变量
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = myleddev_open,
    .release = myleddev_release,
    .read = myleddev_read,
    .write = myleddev_write,
};



/* 模块入口函数 */
static int __init myleddev_init(void)
{
	int ret = -1;

	printk("my_chrdevbase_init\r\n"); //这里要用/r/n，不然会导致linux的命令界面卡住
	
	// 1.动态分配设备号(会自动注册!)
	ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
	if(ret < 0){
		printk(KERN_ERR "alloc_chrdev_region error!\r\n");
		return ret;
	}else{
		printk("%s init:major:%d, minor:%d\r\n", DEVICE_NAME, MAJOR(dev_num), MINOR(dev_num));
	}

	// 2.初始化 cdev 结构体
    cdev_init(&my_cdev, &fops);
    my_cdev.owner = THIS_MODULE;
    // 注册字符设备
    ret = cdev_add(&my_cdev, dev_num, 1);	
	if(ret < 0){
		/* 字符设备注册失败, 需要进行处理 */
		unregister_chrdev_region(dev_num, 1); //如果注册失败，就先释放之前的设备
		led_unmap();
		printk(KERN_ERR "Failed to add cdev\n"); //打印错误信息
		return ret;
	}

	// 3.创建设备节点
	// 3.1.创建class类
	my_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(my_class)) {
        cdev_del(&my_cdev);
        unregister_chrdev_region(dev_num, 1);
        printk(KERN_ERR "Failed to create class\n");
        return PTR_ERR(my_class);
    }

	// 3.2.创建device类
	my_device = device_create(my_class, NULL, dev_num, NULL, DEVICE_NAME);
	if (IS_ERR(my_device)) {
		class_destroy(my_class);
		cdev_del(&my_cdev);
        unregister_chrdev_region(dev_num, 1);
        printk(KERN_ERR "Failed to create device\n");
        return PTR_ERR(my_device);
	}

	return 0;
}

/* 模块出口函数 */
static void __exit myleddev_exit(void)
{
    printk("myleddev_exit\r\n");
	/* 出口函数具体内容 */

	//注销device类
	device_destroy(my_class, dev_num);

	//注销class类
	class_destroy(my_class);

	// 注销字符设备驱动
	cdev_del(&my_cdev);

	/* 注销设备号 */
	unregister_chrdev_region(dev_num, 1);
}

/* 将上面两个函数指定为驱动的入口和出口函数 */
module_init(myleddev_init);
module_exit(myleddev_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("test01");
