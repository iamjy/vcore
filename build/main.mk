#
MODULE_CNM_9X0   = vcore/cnm_9x0
MODULE_CNM_8550J = vcore/cnm_8550j
MODULE_PNG       = vcore/png

BUILD_MODULE_CNM_9X0   = build_module_cnm_9x0
BUILD_MODULE_CNM_8550J = build_module_cnm_8550j
BUILD_MODULE_PNG       = build_module_png

all: $(BUILD_MODULE_CNM_9X0) $(BUILD_MODULE_CNM_8550J) $(BUILD_MODULE_PNG)
build_module_cnm_9x0:
	$(MAKE) -C $(MODULE_CNM_9X0)
build_module_cnm_8550j:
	$(MAKE) -C $(MODULE_CNM_8550J)
build_module_png:
	$(MAKE) -C $(MODULE_PNG)

clean:
	$(MAKE) -C $(MODULE_CNM_9X0) clean
	$(MAKE) -C $(MODULE_CNM_8550J) clean
	$(MAKE) -C $(MODULE_PNG) clean
