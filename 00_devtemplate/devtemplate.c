#include <linux/module.h> /* 模块API相关的头文件 */
#include <linux/pci.h> /* 内核打印 printk 的头文件 */

/* 模块入口函数 */
static int __init xxx_init(void)
{
	printk("my_chrdevbase_init\r\n"); //这里要用/r/n，不然会导致linux的命令界面卡住

	return 0;
}

/* 模块出口函数 */
static void __exit xxx_exit(void)
{
    printk("xxx_exit\r\n");
/* 出口函数具体内容 */
}
/* 将上面两个函数指定为驱动的入口和出口函数 */
module_init(xxx_init);
module_exit(xxx_exit);


MODULE_LICENSE("GPL");
