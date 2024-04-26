# ipu7-drivers

This repository supports MIPI cameras through the IPU7 on Intel Lunar Lake platform.
There are 4 repositories that provide the complete setup:

- https://github.com/intel/ipu7-drivers - kernel drivers for the IPU and sensors
- https://github.com/intel/ipu7-camera-bins - IPU firmware and proprietary image processing libraries
- https://github.com/intel/ipu7-camera-hal - HAL for processing of images in userspace
- https://github.com/intel/icamerasrc/tree/icamerasrc_slim_api (branch:icamerasrc_slim_api) - Gstreamer src plugin


## Content of this repository:
- IPU7 kernel driver
- Kernel patches needed

## Dependencies
- INTEL_SKL_INT3472 and OV13B10 should be enabled.

## Build instructions:
Three ways are available:
1. Build with kernel source code tree (in-tree build)
2. Build with kernel headers only (out-of-tree build)
3. Build and install by dkms (DKMS build)

### 1. In-tree build
- Tested with kernel v6.4 only
- Check out kernel
- Apply patches under patch/
- Copy `drivers` and `include` folders to kernel source **(except Kconfig & Makefile at drivers/media/pci/intel as they are modified by patches in previous step. You can delete them before you copy folders.)**.

- Enable the following settings in .config
	```conf
	CONFIG_VIDEO_INTEL_IPU7=m
	CONFIG_IPU_ISYS_BRIDGE=y
	CONFIG_VIDEO_OV13B10=m
	CONFIG_PINCTRL_INTEL_PLATFORM=m
	CONFIG_INTEL_SKL_INT3472=m
	```

### 2. Out-of-tree build
- Requires kernel headers installed on build machine
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
