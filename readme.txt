================================================================================
Processor Companion Utility (PCU) Software - MSPM0
Copyright (c) 2024, Texas Instruments Incorporated
Revision 0.13 - 2024 April 14
================================================================================

This package contains the source and build scripts for the
processor companion utility (PCU) software.  PCU software
is a collection of modules for MSPM0 microcontrollers
which enable integration of various board level
management functions into one programmable device.

================================================================================
Target platforms
================================================================================

PocketBeagle 2
 - PCU running on MSPM0L1105TRGER device

================================================================================
Currently implemented functions
================================================================================

EEPROM emulation (AT24C256 compatible)
 - Available at I2C address 0x50
 - 16kB memory, 2 address bytes, 64B page write size; accessed as a 32kB
   24c256 the contents appear twice.  The layout fits the 32kB MSPM0L1305
   and runs unchanged on the MSPM0L1306.  A 32kB EEPROM layout for the
   L1306 only is available by building with FW_LAYOUT_64K (see
   source/board/fw_layout.h).
 - Contents stored as-is in flash (layout in source/board/fw_layout.h);
   writes collect in a RAM copy of one 1kB flash sector that is written
   back when a write moves to another sector, when the end of the sector is
   written, or once writes stop for a wake timer tick (0.5-1s).  Rewrites
   that need an erase go through a scratch sector and a log, so a reset or
   power loss leaves each sector either old or new.
 - The target NAKs while a sector is written back (tens of ms, up to a few
   hundred ms on worn flash); with Linux use at24.write_timeout=1000.

Analog to digital converter, 8 channels @ 12 bit resolution, either
 - AD7291 compatible (mspm0-adc-eeprom.txt, mspm0-adc.txt), or
 - AD7998 compatible (mspm0-ad7998-eeprom.txt, mspm0-ad7998.txt), for
   kernels that ship the ad799x driver but not ad7291 (e.g. Raspberry Pi).
   Instantiate with compatible "adi,ad7998" or new_device "ad7998 <addr>".
   The ADC converts against VDDA, which matches the AD7998 using its
   supply as reference, so give the node a vcc-supply of the board's 3.3V
   for in_voltage_scale to work.  The driver provides a triggered buffer
   but no trigger is set up; read the in_voltageN_raw files.

Flashing: firmware with the layout descriptor at 0xC0 (all builds from this
tree) expects the EEPROM image as raw bytes at the descriptor's address; a
flasher that knows the descriptor reads it from the firmware image.

================================================================================
Dependencies
================================================================================

MSPM0-SDK: This project uses the mspm0-sdk.  The project source directory
includes the necessary modules from the mspm0-sdk; there is no requirement
to install the mspm0-sdk just to build the software.  However, it is
recommended to install the mspm0-sdk and SysConfig to use SysConfig 
for modifying the device configuration.

================================================================================
Build instructions
================================================================================

To build the software down to a hex file, a Makefile is provided.  Dependencies
include the TI ARM CLANG compiler tool chain, available from TI.com below:
https://www.ti.com/tool/download/ARM-CGT-CLANG

By default, the Makefile.defs will look for the compiler toolchain in
the home dir of the build user (~/ti/ti-cgt-armllvm_3.2.2.LTS by default)

If you have your toolchain located elsewhere, update $(TOOL_DIR) in
the Makefile.defs file to match your install location.

With the tool chain installed and referenced, "make" may be run from the root
directory of this package.  The expected output files include:

pcu_pocketbeagle2.txt: This is the TI-TXT format hex file for download

pcu_pocketbeagle2.out: This is the .out file (not required)

pcu_pocketbeagle2.map: This is the map file for reference (not required)

================================================================================
Software update instructions
================================================================================

The target MSPM0 device (MSPM0L1105TRGER on PocketBeagle2) supports software
update via its ROM I2C boot strap loader.  PocketBeagle2 includes a Python
script for downloading the pcu_pocketbeagle2.txt build file generated
by the build process above to the target MSPM0 device.  For now,
simply 'scp' the pcu_pocketbeagle2.txt to the target Sitara MPU on PocketBeagle2
and use the m0_bsl.py flasher script to download the firmware.  As it stands,
the target file in the m0_bsl.py script should be updated to 
pcu_pocketbeagle2.txt to use the output of this build script.

================================================================================
Tests
================================================================================

A test directory is included which provides some basic test vectors for the
PCU software package.  The general use model is that these tests are C apps
that may be compiled and run on an Arm Cortex-A target running Linux.

================================================================================
Integration with embedded Linux 
================================================================================

Example device tree overlay source (.dts) for EEPROM, in this case on I2C-2:
--------------------------------------------------------------------------------
/dts-v1/;
/plugin/;

/ {
	fragment@0 {
		target = <&i2c2>;
		__overlay__ {
			status = "okay";
			clock-frequency = <100000>;
			#address-cells = <1>;
			#size-cells = <0>;
			
			eeprom: eeprom@50 {
				compatible = "24c32";
				reg = <0x50>;
			};
		};
	};
};
--------------------------------------------------------------------------------

Example u-boot kernel command line parameter to allow for longer write timeouts:
--------------------------------------------------------------------------------
at24.write_timeout=1000
--------------------------------------------------------------------------------

Example device tree overlay source (.dts) for ADC, in this case on I2C-2:
--------------------------------------------------------------------------------
/dts-v1/;
/plugin/;

/ {
	fragment@0 {
		target-path = "/";
		__overlay__ {
			adc_vref: fixedregulator@0 {
				compatible = "regulator-fixed";
				regulator-name = "fixed-supply";
				regulator-min-microvolt = <3300000>;
				regulator-max-microvolt = <3300000>;
				regulator-boot-on;
			};
		};
	};

	fragment@1 {
		target = <&i2c2>;
		__overlay__ {
			status = "okay";
			clock-frequency = <100000>;
			#address-cells = <1>;
			#size-cells = <0>;
			
			ad7291: adc@20 {
				compatible = "adi,ad7291";
				reg = <0x20>;
				vref-supply = <&adc_vref>;
			};
		};
	};
};
--------------------------------------------------------------------------------

================================================================================
License
================================================================================

This software is provided "as is" under the BSD-3-Clause license.

================================================================================
Change log
================================================================================

v0.1 - 2024-FEB-26
Initial release with 1kB at24c compatible EEPROM, no ADC

v0.11 - 2024-APR-14
Refactoring of I2C target driver to facilitate responding to two different
I2C target addresses (this will be needed to enable having EEPROM and ADC
functions coexisting on the same I2C bus).

v0.12 - 2024-APR-14
Adding 8-ch 12-bit ADC functionality over I2C bus (address 0x20),
compatible with ad7291 Linux IIO ADC driver.  Not all AD7291
functionality is supported (see limitations) but raw measurements
are functional.

v0.13 - 2024-APR-14
Increasing EEPROM size from 1kB (8kbit) to 4kB (32kbit) at expense
of slightly longer write times in cases where many writes have been
committed to the EEPROM and EEPROM emulation pages need to be swapped.
The benefit of this change is achieving full compatibility with the 
at24c32 driver.  With only 1kB, use of 24c32 would cause sysfs reads
to deliver the EEPROM contents 4 times (reading 1kB back 4 times
because the Linux driver thinks it is working with a 4kB memory).
Right now this seems like the better trade-off.  To counteract the
longer write times after extensive EEPROM usage, it is recommended
to set at24.write_timeout=1000 at the kernel command line (can
be done via uEnv.txt).

Unreleased
EEPROM rewritten: 16kB stored as-is in flash with a RAM sector cache and a
power-fail safe scratch/log rewrite, replacing the TI Type B EEPROM
emulation library.  The layout fits the 32kB MSPM0L1305 as well as the
MSPM0L1306.  Firmware layout descriptor at 0xC0 for the flasher.
AD7998 compatible ADC builds added beside the AD7291 ones.
