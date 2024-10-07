#include <linux/module.h> /* 模块API相关的头文件 */
#include <linux/pci.h> /* 内核打印 printk 的头文件 */
#include <linux/fs.h> /* static struct file_operations 的头文件 */

#define DEVICE_NAME			"chardev"

dev_t my_dev = 0;

int chardev_open(struct inode *inode, struct file *file)
{
	printk("chardev_open\r\n");
	return 0;
}


int chardev_release(struct inode *inode, struct file *file)
{
	printk("chardev_release\r\n");
	return 0;
}

ssize_t chardev_read(struct file *file, char __user *buf, size_t size, loff_t * offset)
{
	printk("chardev_read\r\n");
	return 0;
}

ssize_t chardev_write(struct file *file, const char __user *buf, size_t size, loff_t * offset)
{
	printk("chardev_write\r\n");
	return 0;
}


// 创建一个文件操作结构变量
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = chardev_open,
    .release = chardev_release,
    .read = chardev_read,
    .write = chardev_write,
};



/* 模块入口函数 */
static int __init chardev_init(void)
{
	int ret = -1;

	printk("my_chrdevbase_init\r\n"); //这里要用/r/n，不然会导致linux的命令界面卡住
	
	ret = alloc_chrdev_region(&my_dev, 250, 1, DEVICE_NAME);
	if(ret < 0)
	{
		printk("alloc_chrdev_region error!\r\n");
		return ret;
	}
	else
	{
		printk("%s init:major:%d, minor:%d\r\n", DEVICE_NAME, MAJOR(my_dev), MINOR(my_dev));
	}
	
	// 注册字符设备驱动   设备号     设备名      文件操作结构体
	ret = register_chrdev(MAJOR(my_dev), DEVICE_NAME, &fops);
	// ret = register_chrdev(200, DEVICE_NAME, &fops);
	if(ret < 0)
	{
		/* 字符设备注册失败, 需要进行处理 */
		unregister_chrdev_region(my_dev, 1); //如果注册失败，就先释放之前的设备
		printk("register_chrdev error, ret=%d!\r\n", ret); //打印错误信息
		return ret;
	}

	return 0;
}

/* 模块出口函数 */
static void __exit chardev_exit(void)
{
    printk("chardev_exit\r\n");
	// 注销字符设备驱动
	unregister_chrdev(MAJOR(my_dev), DEVICE_NAME);
	// unregister_chrdev(200, DEVICE_NAME);

	/* 注销设备号 */
	unregister_chrdev_region(my_dev, 1);
/* 出口函数具体内容 */
}

/* 将上面两个函数指定为驱动的入口和出口函数 */
module_init(chardev_init);
module_exit(chardev_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("test01");
