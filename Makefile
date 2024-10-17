SRC := $(shell pwd)

EXTRA_CFLAGS += -I$(SRC)/include
ccflags-y += -Wno-declaration-after-statement
obj-m += tmf882x.o
tmf882x-y += tmf882x_driver.o tmf882x_clock_correction.o tmf882x_mode.o tmf882x_mode_app.o tmf882x_mode_bl.o tmf882x_interface.o intel_hex_interpreter.o


all:
	$(MAKE) -C $(KERNEL_SRC) $(EXTRA_FLAGS) M=$(SRC)
	
modules_install:
	$(MAKE) -C $(KERNEL_SRC) $(EXTRA_FLAGS) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c *.a *.mod
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
