# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2022 Intel Corporation.

KERNELRELEASE ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KERNELRELEASE)/build
MODSRC := $(shell pwd)

export EXTERNAL_BUILD = 1
export CONFIG_VIDEO_INTEL_IPU7 = m
export CONFIG_IPU_BRIDGE = y
export CONFIG_INTEL_IPU_ACPI = m

obj-y += drivers/media/pci/intel/ipu7/
obj-y += drivers/media/platform/intel/
subdir-ccflags-y += -I$(src)/include

subdir-ccflags-$(CONFIG_IPU_BRIDGE) += \
	-DCONFIG_IPU_BRIDGE
subdir-ccflags-$(CONFIG_INTEL_IPU_ACPI) += \
	-DCONFIG_INTEL_IPU_ACPI
subdir-ccflags-y += $(subdir-ccflags-m)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) modules
modules_install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KERNEL_SRC) M=$(MODSRC) modules_install
clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean
