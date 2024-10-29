#include <linux/module.h> /* 模块API相关的头文件 */
#include <linux/pci.h> /* 内核打印 printk 的头文件 */
#include <linux/fs.h> /* static struct file_operations 的头文件 */
#include <linux/cdev.h>//cdev头文件
#include <linux/uaccess.h>//处理用户空间和内核空间数据传递接口的头文件
#include <linux/string.h>//额，linux不能使用普通的string.h,需要使用内核的string.h
#include <linux/of.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/errno.h>

#include "leddev_dt_rk3568.h"
/////////////////////////////////////////////////
/*
	使用GPIO子系统控制GPIO
	1. 需要先向设备树中写入对应的GPIO节点信息
	如，在根节点下添加：
		my_test_led {
		compatible = "my-test-dt-led-rk3568";
		pinctrl-names = "default";

		gpios = <&gpio0 RK_PB0 GPIO_ACTIVE_LOW>;
	};
*/
/////////////////////////////////////////////////


#define DEVICE_NAME			"myleddev"
#define CLASS_NAME			"myleddev_class"

#define MAX_NAME_SIZE		100

//定义一个结构体来管理驱动
typedef struct myled_type{
	dev_t dev_num; //设备号
	struct cdev my_cdev; // cdev结构体
	struct class* my_class;// class类
	struct device* my_device; //device类
	struct file_operations fops; //读写等操作的具体实现
	struct device_node *nd; //使用设备树的方式来写驱动，需要一个设备树节点类来获取设备树信息
	int led_gpio; //led gpio对应的IO编号
}myled_t;

myled_t myled = {0};

int led_on(int gpio_num)
{
	gpio_set_value(gpio_num, 0);
	return 0;
}


int led_off(int gpio_num)
{
	gpio_set_value(gpio_num, 1);
	return 0;
}

int myleddev_open(struct inode *inode, struct file *file)
{
	printk("myleddev_open\r\n");
	
	file->private_data = &myled;

	return 0;
}


int myleddev_release(struct inode *inode, struct file *file)
{
	myled_t *leddev = NULL;
	leddev = file->private_data;
	printk("myleddev_release\r\n");

	//释放gpio资源
	gpio_free(leddev->led_gpio);

	return 0;
}

ssize_t myleddev_read(struct file *file, char __user *buf, size_t len, loff_t * offset)
{
	int bytes_read;
	char read_buf[10] = "for test";

	// 将数据拷贝到用户buf
    bytes_read = copy_to_user(buf, read_buf, len);
	if (bytes_read < 0) {
		printk(KERN_ERR "copy_to_user failed!\r\n");
        return -EFAULT; // 返回错误
    }
	*offset += sizeof(read_buf);

	printk(KERN_INFO "mymyleddev: Read %d bytes\n", bytes_read);
    return bytes_read;
}

ssize_t myleddev_write(struct file *file, const char __user *buf, size_t len, loff_t * offset)
{
	int bytes_written;
	myled_t *leddev = NULL;
	char data_buf[10] = {0};

	leddev = file->private_data;

	//拷贝用户空间数据到内核空间
	bytes_written = copy_from_user(data_buf, buf, len);//copy_from_user的返回值是未成功拷贝的字节数
	if(bytes_written < 0) {
		printk(KERN_ERR "copy_from_user failed!\r\n");
		return -EFAULT; // 返回错误
	}
	if(strcmp(data_buf, "on") == 0) {
		//on
		led_on(leddev->led_gpio);
	} else if (strcmp(data_buf, "off") == 0){
		//off
		led_off(leddev->led_gpio);
	} else {
		printk(KERN_ERR "unknow parameter!\r\n");
		return -EINVAL;
	}

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


static int my_myled_init(myled_t *myled)
{
	int ret = 0;
	const char *str;
	if(myled == NULL){
		printk(KERN_ERR "my_myled_init:myled=null!\r\n");
		return -EINVAL;
	}

	myled->fops = fops; //绑定fops操作符

	// 读取设备树,根据设备树节点来获取IO信息
	//of操作有很多,这里通过节点名来获取
	myled->nd = of_find_compatible_node(NULL, NULL, "my-test-dt-led-rk3568");
	if(myled->nd == NULL){
		printk(KERN_ERR "find my-test-dt-led-rk3568 dt node failed!\r\n");
		return -EFAULT;
	}

	//检测状态是否为okey
	ret = of_property_read_string(myled->nd, "status", &str);
	if(strcmp(str, "okay") != 0){
		printk(KERN_ERR "status != okay!\r\n");
		return -EFAULT;
	}

	//获取设备树中的GPIO编号
	myled->led_gpio = of_get_named_gpio(myled->nd, "gpios", 0);
	if(myled->led_gpio < 0){
		printk(KERN_ERR "myled->led_gpio = of_get_named_gpio(myled->nd, gpios);\r\n");
		return -EFAULT;
	}

	// 向gpio子系统申请IO
	ret = gpio_request(myled->led_gpio, "my-test-led");
	if(ret < 0){
		printk(KERN_ERR "gpio_request failed!\r\n");
		return -EFAULT;
	}

	// 设置GPIO默认状态
	ret = gpio_direction_output(myled->led_gpio, 1);//默认设置为高电平
	if(ret < 0){
		gpio_free(myled->led_gpio);
		printk(KERN_ERR "gpio_direction_output failed!\r\n");
		return -EFAULT;
	}

	return 0;
}


/* 模块入口函数 */
static int __init myleddev_init(void)
{
	int ret = -EFAULT;

	printk("my_chrdevbase_init\r\n"); //这里要用/r/n,不然会导致linux的命令界面卡住

	if(my_myled_init(&myled) < 0)//初始化myled的地方
	{
		printk(KERN_ERR "myleddev_init failed! : my_myled_init(&myled) < 0\r\n");
		return -1;
	}

	// 1.动态分配设备号(会自动注册!)
	ret = alloc_chrdev_region(&myled.dev_num, 0, 1, DEVICE_NAME);
	if(ret < 0){
		printk(KERN_ERR "alloc_chrdev_region error!\r\n");
		return ret;
	}else{
		printk("%s init:major:%d, minor:%d\r\n", DEVICE_NAME, MAJOR(myled.dev_num), MINOR(myled.dev_num));
	}

	// 2.初始化 cdev 结构体
    cdev_init(&myled.my_cdev, &myled.fops);
    myled.my_cdev.owner = THIS_MODULE;
    // 注册字符设备
    ret = cdev_add(&myled.my_cdev, myled.dev_num, 1);
	if(ret < 0){
		/* 字符设备注册失败, 需要进行处理 */
		unregister_chrdev_region(myled.dev_num, 1); //如果注册失败，就先释放之前的设备
		printk(KERN_ERR "Failed to add cdev\r\n"); //打印错误信息
		return ret;
	}

	// 3.创建设备节点
	// 3.1.创建class类
	myled.my_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(myled.my_class)) {
        cdev_del(&myled.my_cdev);
        unregister_chrdev_region(myled.dev_num, 1);
        printk(KERN_ERR "Failed to create class\r\n");
        return PTR_ERR(myled.my_class);
    }

	// 3.2.创建device类
	myled.my_device = device_create(myled.my_class, NULL, myled.dev_num, NULL, DEVICE_NAME);
	if (IS_ERR(myled.my_device)) {
		class_destroy(myled.my_class);
		cdev_del(&myled.my_cdev);
        unregister_chrdev_region(myled.dev_num, 1);
        printk(KERN_ERR "Failed to create device\r\n");
        return PTR_ERR(myled.my_device);
	}

	return 0;
}

/* 模块出口函数 */
static void __exit myleddev_exit(void)
{
    printk("myleddev_exit\r\n");
	/* 出口函数具体内容 */

	//注销device类
	device_destroy(myled.my_class, myled.dev_num);

	//注销class类
	class_destroy(myled.my_class);

	// 注销字符设备驱动
	cdev_del(&myled.my_cdev);

	/* 注销设备号 */
	unregister_chrdev_region(myled.dev_num, 1);
}

/* 将上面两个函数指定为驱动的入口和出口函数 */
module_init(myleddev_init);
module_exit(myleddev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("test03");
