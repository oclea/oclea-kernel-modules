obj-m += pwm-gpio.o

KDIR := $(KERNEL_HEADERS_DIR)
PWD := $(shell pwd)

all:
	make -C $(KERNEL_HEADERS_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_HEADERS_DIR) M=$(PWD) clean

