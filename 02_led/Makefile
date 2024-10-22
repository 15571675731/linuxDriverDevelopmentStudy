
#指定内和路径，编译模块需要引用内核的一些库
# KERNELDIR := /home/luoxl/linux/sdk/linux/
KERNELDIR := /home/luoxl/rk3568/LinuxSDK/sdk/rk356x_linux_release_v1.3.1_20221120/kernel

#指定当前路径 通过pwd来获得
CURRENT_PATH := $(shell pwd)

obj-m := leddev.o

# CP_PATH := /home/luoxl/linux/nfs/rootfs/lib/modules/4.1.15
CP_PATH := /mnt/hgfs/share_dir_rk3568/app/dev_test

APPCC := aarch64-linux-gnu-gcc -g
APP := testapp

#默认 build 选择 kernel_modules
build: kernel_modules app

#kernel_modules 模块
kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules
	cp *.ko $(CP_PATH)
#clean模块
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean

app:
	$(APPCC) testapp.c -o $(APP)
	cp $(APP) $(CP_PATH)