CROSS = arm-linux-gnueabihf-

MODULE := png_module
obj-m := $(MODULE).o

$(MODULE)-y += odin_mpng.o
$(MODULE)-y += odin_mpng_dec.o
$(MODULE)-y += mpng.o

KDIR := $(MTEKROOT_KDIR)

all:
ifndef VCORE_DIR
	@echo ""
	@echo "Please specify vcore source directory!"
	@echo "#####################################################################################################"
	@echo "# Usage                                                                                             #"
	@echo "#    make VCORE_DIR=[WORK_DIR/VCORE_DIR] MTEKROO_KDIR=[WORK_DIR]/mtekroot/output/build/linux-z3-itb #"
	@echo "#####################################################################################################"
	@echo ""
	@exit 1
endif
ifndef MTEKROOT_KDIR
	@echo ""
	@echo "Please specify kernel directory!"
	@echo "#####################################################################################################"
	@echo "# Usage                                                                                             #"
	@echo "#    make VCORE_DIR=[WORK_DIR/VCORE_DIR] MTEKROO_KDIR=[WORK_DIR]/mtekroot/output/build/linux-z3-itb #"
	@echo "#####################################################################################################"
	@echo ""
	@exit 1
endif

	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS) M=$$PWD
	$(MAKE) -C $(KDIR) ARCH=arm CROSS_COMPILE=$(CROSS) INSTALL_MOD_PATH=../../target M=$$PWD modules_install

clean:
	@rm -rf *.ko
	@rm -rf *.o
	@rm -rf *.mod.c
	@rm -rf *.cmd
	@rm -rf *.o.cmd
	@rm -rf .*.o.cmd
	@rm -rf .*.ko.cmd

ccflags-y := -I$(VCORE_DIR)
ccflags-y += -I$(MTEKROOT_KDIR)/drivers/media/platform/odin-vcodec
