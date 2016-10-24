CROSS = arm-linux-gnueabihf-

MODULE := cnm_8550j_module
obj-m := $(MODULE).o

$(MODULE)-y += cnm_ddk/jdi/linux/jdi.o
$(MODULE)-y += cnm_ddk/jpuapi/jpuapi.o
$(MODULE)-y += cnm_ddk/jpuapi/jpuapifunc.o
$(MODULE)-y += coda8550j.o
$(MODULE)-y += coda8550j_enc.o
$(MODULE)-y += coda8550j_dec.o

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
	$(MAKE) -C cnm_ddk clean
	$(MAKE) -C cnm_ddk/jdi/linux clean
	$(MAKE) -C cnm_ddk/jpuapi clean

ccflags-y := -I$(VCORE_DIR)
ccflags-y += -I$(MTEKROOT_KDIR)/drivers/media/platform/odin-vcodec
