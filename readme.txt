================================================================================
Processor Companion Utility (PCU) Software - MSPM0
Copyright (c) 2024, Texas Instruments Incorporated
Revision 0.11 - 2024 April 14
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

EEPROM emulation (AT24C I2C EEPROM compatible)
 - Available at I2C address 0x50
 - 1kB (8kbit) memory
 - 32B page write size

================================================================================
Future planned extensions
================================================================================

Analog to digital converter
 - 8 channels @ 12 bit resolution

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
