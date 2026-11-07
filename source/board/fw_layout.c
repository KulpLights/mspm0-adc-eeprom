/*
 *  fw_layout.c
 *
 *  The layout descriptor the flasher reads from the firmware image.  The
 *  linker places the .fwlayout section at FW_LAYOUT_ADDR.
 */

#include "fw_layout.h"

__attribute__((used, retain, section(".fwlayout")))
const fw_layout_t fw_layout =
{
    .magic = FW_LAYOUT_MAGIC,
    .version = FW_LAYOUT_VERSION,
    .length = sizeof(fw_layout_t),
    .eepromFormat = FW_EEPROM_FORMAT_RAW,
    .eepromAddr = FW_EEPROM_DATA_ADDR,
    .eepromSize = FW_EEPROM_DATA_SIZE,
    .oledImageAddr = FW_OLED_IMAGE_ADDR,
    .oledImageSize = FW_OLED_IMAGE_SIZE,
};
