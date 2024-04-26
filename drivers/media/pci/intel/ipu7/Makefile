# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2017 - 2024 Intel Corporation.

ifneq ($(EXTERNAL_BUILD), 1)
srcpath := $(srctree)
endif

intel-ipu7-objs				+= ipu.o \
					   ipu-bus.o \
					   ipu-dma.o \
					   ipu-mmu.o \
					   ipu-buttress.o \
					   ipu-cpd.o \
					   ipu-syscom.o \
					   ipu-boot.o

ifdef CONFIG_IPU_ISYS_BRIDGE
intel-ipu7-objs				+= ../cio2-bridge.o
endif

obj-$(CONFIG_VIDEO_INTEL_IPU7)		+= intel-ipu7.o

intel-ipu7-isys-objs			+= ipu-isys.o \
					   ipu-isys-csi2.o \
					   ipu7-isys-csi-phy.o \
					   ipu-isys-csi2-be-soc.o \
					   ipu-fw-isys.o \
					   ipu-isys-video.o \
					   ipu-isys-queue.o \
					   ipu-isys-subdev.o

ifdef CONFIG_VIDEO_INTEL_IPU_MGC
intel-ipu7-isys-objs                    += ipu-isys-tpg.o
endif

obj-$(CONFIG_VIDEO_INTEL_IPU7)		+= intel-ipu7-isys.o

intel-ipu7-psys-objs			+= ipu-psys.o \
					   ipu7-psys.o \

intel-ipu7-psys-objs			+= ipu-fw-psys.o

ifeq ($(CONFIG_COMPAT),y)
intel-ipu7-psys-objs			+= ipu-psys-compat32.o
endif

obj-$(CONFIG_VIDEO_INTEL_IPU7)		+= intel-ipu7-psys.o

ccflags-y += -I$(srcpath)/$(src)/../../../../../include/
ifdef CONFIG_IPU_ISYS_BRIDGE
ccflags-y += -I$(srcpath)/$(src)/../
endif
ccflags-y += -I$(srcpath)/$(src)/platform
ccflags-y += -I$(srcpath)/$(src)/abi/ia_gofo_boot_abi/
ccflags-y += -I$(srcpath)/$(src)/abi/ia_gofo_common_abi/
ccflags-y += -I$(srcpath)/$(src)/abi/ia_gofo_syscom_abi/
ccflags-y += -I$(srcpath)/$(src)/abi/ipu_msg_abi/
ccflags-y += -I$(srcpath)/$(src)/abi/ipu_isys_abi/include
ccflags-y += -I$(srcpath)/$(src)/abi/ipu_config_abi/
ccflags-y += -I$(srcpath)/$(src)/abi/ipu_insys_config_abi/
ccflags-y += -I$(srcpath)/$(src)/abi/ipu_psys_config_abi/
ccflags-y += -I$(srcpath)/$(src)/
