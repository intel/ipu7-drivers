# ipu7-drivers

This repository supports MIPI cameras through the IPU7 on Intel Lunar Lake platform.
There are 4 repositories that provide the complete setup:

- https://github.com/intel/ipu7-drivers - kernel drivers for the IPU and sensors
- https://github.com/intel/ipu7-camera-bins - IPU firmware and proprietary image processing libraries
- https://github.com/intel/ipu7-camera-hal - HAL for processing of images in userspace
- https://github.com/intel/icamerasrc/tree/icamerasrc_slim_api (branch:icamerasrc_slim_api) - Gstreamer src plugin


## Content of this repository
- IPU7 kernel driver
- Kernel patches needed

## Dependencies
- Kernel config IPU_BRIDGE should be enabled when kernel version >= v6.6.
- Kernel config INTEL_SKL_INT3472 should be enabled if camera sensor driver is using discrete power control logic, like ov13b10.c.
- The camera sensor drivers you are using should be enabled in kernel config.
- For other camera sensor drivers and related ipu-bridge changes, please [go to IPU6 release repository](https://github.com/intel/ipu6-drivers).

## Build instructions:
Three ways are available:
1. Build with kernel source code tree (in-tree build)
2. Build with kernel headers only (out-of-tree build)
3. Build and install by dkms (DKMS build)

### 1. In-tree build
- Tested with kernel v6.8 ~ v6.11-rc2
1. Check out kernel source code
2. Patch kernel source code, using patches under `patch/<kernel-version>` and `patch/<kernel-version>/in-tree-build` depending on what you need.
3. Copy `drivers` and `include` folders to kernel source code.
4. Enable the following settings in .config:
	```conf
	CONFIG_VIDEO_INTEL_IPU7=m
	CONFIG_IPU_BRIDGE=m
	```
	And other components, depending on what you need:
	```conf
	CONFIG_PINCTRL_INTEL_PLATFORM=m
	CONFIG_INTEL_SKL_INT3472=m
	CONFIG_VIDEO_OV13B10=m
	CONFIG_VIDEO_OV02C10=m
	CONFIG_VIDEO_OV05C10=m
	```
5. Build you kernel

### 2. Out-of-tree build
- Requires kernel headers installed on build machine
- Requires dependencies enabled in kernel, and patches under `patch/<kernel-version>` (not `in-tree-build`) applied to the kernel you will run the modules on
- To build and install:
	```sh
	make -j`nproc` && sudo make modules_install && sudo depmod -a
	```

### 3. Build with dkms

- Register, build and auto install:
	```sh
	sudo dkms add .
	sudo dkms autoinstall ipu7-drivers/0.0.0
	```
