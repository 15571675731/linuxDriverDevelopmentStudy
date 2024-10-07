
#指定内和路径，编译模块需要引用内核的一些库
KERNELDIR := /home/luoxl/linux/sdk/linux/

#指定当前路径 通过pwd来获得
CURRENT_PATH := $(shell pwd)

obj-m := chardev.o


#默认 build 选择 kernel_modules
build: kernel_modules

#kernel_modules 模块
kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
	cp *.ko /home/luoxl/linux/nfs/rootfs/lib/modules/4.1.15
#clean模块
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
