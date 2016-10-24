CROSS := arm-linux-gnueabihf-

MODULE := cnm_9x0_module
obj-m := $(MODULE).o

$(MODULE)-y += cnm_9x0.o
$(MODULE)-y += boda950/boda950.o
$(MODULE)-y += boda950/boda950_dec.o
$(MODULE)-y += boda950/boda950_ppu.o
$(MODULE)-y += cnm/cnm.o
$(MODULE)-y += cnm/cnm_enc.o
$(MODULE)-y += cnm/cnm_dec.o
$(MODULE)-y += cnm/cnm_ppu.o
$(MODULE)-y += util/reorder_queue.o
$(MODULE)-y += coda980/coda980.o
$(MODULE)-y += coda980/coda980_enc.o
$(MODULE)-y += coda980/coda980_dec.o
$(MODULE)-y += coda980/coda980_ppu.o
$(MODULE)-y += cnm_ddk/vdi/linux/vdi.o
$(MODULE)-y += cnm_ddk/vdi/linux/vdi_osal.o
$(MODULE)-y += cnm_ddk/vpuapi/vpuapi.o
$(MODULE)-y += cnm_ddk/vpuapi/vpuapifunc.o

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
	@rm -rf *.o.cmd
	@rm -rf .*.o.cmd
	@rm -rf .*.ko.cmd
	$(MAKE) -C boda950 clean
	$(MAKE) -C cnm clean
	$(MAKE) -C cnm_ddk clean
	$(MAKE) -C cnm_ddk/vdi/linux clean
	$(MAKE) -C cnm_ddk/vpuapi clean
	$(MAKE) -C coda980 clean
	$(MAKE) -C util clean

ccflags-y := -I$(VCORE_DIR)
ccflags-y += -I$(MTEKROOT_KDIR)/drivers/media/platform/odin-vcodec
