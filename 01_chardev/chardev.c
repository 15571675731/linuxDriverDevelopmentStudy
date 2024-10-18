#include <linux/module.h> /* 模块API相关的头文件 */
#include <linux/pci.h> /* 内核打印 printk 的头文件 */
#include <linux/fs.h> /* static struct file_operations 的头文件 */
#include <linux/cdev.h>//cdev头文件
#include <linux/uaccess.h>//处理用户空间和内核空间数据传递接口的头文件

#define DEVICE_NAME			"chardev"

static dev_t dev_num = 0;
static struct cdev my_cdev;

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
	
	//动态分配设备号(会自动注册!)
	ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
	if(ret < 0){
		printk(KERN_ERR "alloc_chrdev_region error!\r\n");
		return ret;
	}else{
		printk("%s init:major:%d, minor:%d\r\n", DEVICE_NAME, MAJOR(dev_num), MINOR(dev_num));
	}

	// 初始化 cdev 结构体
    cdev_init(&my_cdev, &fops);
    my_cdev.owner = THIS_MODULE;
    // 注册字符设备
    ret = cdev_add(&my_cdev, dev_num, 1);	
	// 注册字符设备驱动   设备号     设备名      文件操作结构体
	if(ret < 0)
	{
		/* 字符设备注册失败, 需要进行处理 */
		unregister_chrdev_region(dev_num, 1); //如果注册失败，就先释放之前的设备
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
	cdev_del(&my_cdev);

	/* 注销设备号 */
	unregister_chrdev_region(dev_num, 1);
/* 出口函数具体内容 */
}

/* 将上面两个函数指定为驱动的入口和出口函数 */
module_init(chardev_init);
module_exit(chardev_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("test01");
