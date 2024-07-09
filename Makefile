# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2022 Intel Corporation.

KERNELRELEASE ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KERNELRELEASE)/build
MODSRC := $(shell pwd)

export EXTERNAL_BUILD = 1
export CONFIG_VIDEO_INTEL_IPU7 = m
# export CONFIG_IPU_ISYS_BRIDGE = n

obj-y += drivers/media/pci/intel/ipu7/
subdir-ccflags-y += -I$(src)/include

export CONFIG_VIDEO_OV02C10 = m
obj-y += drivers/media/i2c/

subdir-ccflags-$(CONFIG_IPU_ISYS_BRIDGE) += \
	-DCONFIG_IPU_ISYS_BRIDGE=1
subdir-ccflags-y += $(subdir-ccflags-m)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) modules
modules_install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KERNEL_SRC) M=$(MODSRC) modules_install
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean
