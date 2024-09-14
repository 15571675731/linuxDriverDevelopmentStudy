
#指定内和路径，编译模块需要引用内核的一些库
KERNELDIR := /home/luoxl/rk3568/LinuxSDK/sdk/rk356x_linux_release_v1.3.1_20221120/kernel

#指定当前路径 通过pwd来获得
CURRENT_PATH := $(shell pwd)

obj-m := devtemplate.o


#默认 build 选择 kernel_modules
build: kernel_modules

#kernel_modules 模块
kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
	cp *.ko /mnt/hgfs/share_dir_rk3568/module/
#clean模块
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
