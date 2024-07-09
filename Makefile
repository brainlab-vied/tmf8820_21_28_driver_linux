ifneq ($(KERNELRELEASE),)
#kbuild part of Makefile
include Kbuild
else
SRC := $(shell pwd)

EXTRA_CFLAGS += -I$(SRC)/include
ccflags-y += -Wno-declaration-after-statement
obj-m += tmf882x.o
tmf882x-y += tmf882x_driver.o tmf882x_clock_correction.o tmf882x_mode.o tmf882x_mode_app.o tmf882x_mode_bl.o tmf882x_interface.o intel_hex_interpreter.o


#normal Makefile
all:
	$(MAKE) -C $(KERNEL_SRC) $(EXTRA_FLAGS) M=$(SRC)

sign:
	$(SIGN_SCRIPT) sha512 $(KERNEL_SRC)/signing_key.priv $(KERNEL_SRC)/signing_key.x509 $(DEVICE_NAME).ko

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
	
modules_install:
	$(MAKE) -C $(KERNEL_SRC) $(EXTRA_FLAGS) M=$(SRC) modules_install

endif
