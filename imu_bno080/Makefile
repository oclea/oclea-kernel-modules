obj-m += imu_bno080.o
imu_bno080-objs := bno080_core.o bno080_api.o bno080_i2c.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) KBUILD_EXTRA_SYMBOLS=$(SRC)/../../recipe-sysroot/usr/include/ambarella-hwtimer/Module.symvers

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
