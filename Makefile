ifneq ($(KERNELRELEASE),)

obj-m :=ds18b20.o
else
KDIR :=/lib/modules/`uname -r`/build
all:
	make -C	$(KDIR) M=$(PWD) modules
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers
endif
