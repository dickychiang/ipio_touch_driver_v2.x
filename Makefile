# include header files for MTK
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/oreo/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

ccflags-y += -Wall

# Build method
BUILD_MODULE := n

ifeq ($(BUILD_MODULE),n)
	obj-y += plat_mtk.o probe.o
else
	obj-m += ilitek.o
	ilitek-y := plat_mtk.o probe.o

KERNEL_DIR= /home/likewise-open/ILI/1061279/workplace/rk3288_sdk/kernel
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
endif
