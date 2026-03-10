# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2022 Intel Corporation.

KERNELRELEASE ?= $(shell uname -r)
KERNEL_SRC ?= /lib/modules/$(KERNELRELEASE)/build
MODSRC := $(shell pwd)
KERNEL_VERSION := $(shell echo $(KERNELRELEASE) | sed 's/[^0-9.]*\([0-9.]*\).*/\1/')

version_lt = $(shell \
    v1=$(1); \
    v2=$(2); \
    IFS='.'; \
    set -- $$v1; i=$${1:-0}; j=$${2:-0}; k=$${3:-0}; \
    set -- $$v2; a=$${1:-0}; b=$${2:-0}; c=$${3:-0}; \
    if [ "$$i" -lt "$$a" ]; then \
        echo "true"; \
    elif [ "$$i" -eq "$$a" ] && [ "$$j" -lt "$$b" ]; then \
        echo "true"; \
    elif [ "$$i" -eq "$$a" ] && [ "$$j" -eq "$$b" ] && [ "$$k" -lt "$$c" ]; then \
        echo "true"; \
    else \
        echo "false"; \
    fi)

KV_IPU7_ISYS := 6.17.0

export EXTERNAL_BUILD = 1
export CONFIG_VIDEO_INTEL_IPU7 = m
export CONFIG_IPU_BRIDGE = y

ifeq ($(call version_lt,$(KERNEL_VERSION),$(KV_IPU7_ISYS)),true)
obj-y += drivers/media/pci/intel/ipu7/
else
obj-y += drivers/media/pci/intel/ipu7/psys/
endif

# For ACPI build: add platform drivers
ifdef BUILD_INTEL_IPU_ACPI
export CONFIG_INTEL_IPU_ACPI = m
obj-y += drivers/media/platform/intel/
subdir-ccflags-$(CONFIG_INTEL_IPU_ACPI) += \
	-DCONFIG_INTEL_IPU_ACPI
endif

subdir-ccflags-y += -I$(src)/include

subdir-ccflags-$(CONFIG_IPU_BRIDGE) += \
	-DCONFIG_IPU_BRIDGE

subdir-ccflags-y += $(subdir-ccflags-m)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) modules

all-acpi:
	$(MAKE) BUILD_INTEL_IPU_ACPI=1 -C $(KERNEL_SRC) M=$(MODSRC) modules

modules_install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KERNEL_SRC) M=$(MODSRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(MODSRC) clean
