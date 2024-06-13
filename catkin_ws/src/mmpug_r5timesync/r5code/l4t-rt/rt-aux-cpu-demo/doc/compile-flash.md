Compiling and Flashing {#rt-compiling}
======================

@tableofcontents

This Document describes the steps to build and flash this project

# TOOLCHAIN PREREQUISITE#

You must download the external Toolchain. NVIDIA does not distribute this toolchain.

https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update

# BUILD #

Set the appropriate paths for the below variables:
```
export TOP=<path to root directory where rt-aux-cpu-demo, hwinc and freertos related sources/directories reside>
export CROSS_COMPILE=<path to installed cross compiler>/gcc-arm-none-eabi-4_8-2014q3/bin/arm-none-eabi-
export OUTDIR=${TOP}/out
export FREERTOS_DIR=${TOP}/FreeRTOSV8.1.2/FreeRTOS/Source
export FREERTOS_COMMON_DIR=${TOP}/freertos-common

cd rt-aux-cpu-demo
```

## To build for Jetson AGX ##

	make TARGET=t19x

## To build for Jetson TX2 (the default) ##

	make TARGET=t18x or
	make

This should result in a `$(OUTDIR)/spe.bin` binary.

## To build doxygen documents ##

	make documents

This should generate doc/rt_html directory, open index.html from it to navigate
through documents.

## To clean the build artifacts ##

	make clean; To clean spe firmware object files
	make doc_clean; To clean doxygen generated files

# FLASH

1. Back up the original copies of `spe.bin` and `spe_t194.bin` located in the
following directory:

    Linux_for_Tegra/bootloader/

2. Copy the generated `${OUTDIR}/spe.bin` to the following locating, depending on your target.
   - For Jetson TX2, copy it to:

         Linux_for_Tegra/bootloader/spe.bin
   - For the Jetson AGX target, copy it to:

         Linux_for_Tegra/bootloader/spe_t194.bin

   The `Linux_for_Tegra` directory is part of the extracted L4T build that
   you used to flash the Jetson device.
3. Use the command below to just flash this firmware for Cortex-R5 partition:

       sudo ./flash.sh -k spe-fw jetson-<tx2 or xavier> mmcblk0p1
