/*
 *  fw_layout.h
 *
 *  Flash layout of the MSPM0L130x firmware image.
 *
 *  Flash is 1KB sectors.  Sectors erased on every EEPROM flush (scratch and
 *  log) must be in the 100k-cycle region: all of flash on the 32KB L1305,
 *  the lower 32KB on the 64KB L1306 (its upper 32KB is rated for 10k).
 *
 *  The CPU can't read the last 8 bytes of flash: the prefetcher reads a
 *  flash word ahead, past the end, and hard faults (datasheet table 8-4,
 *  note 3).  Nothing that is read may sit there.
 *
 *  Default layout: fits the 32KB MSPM0L1305 and runs unchanged on the
 *  L1306 (its upper 32KB is then unused).  The log is the L1305's top
 *  sector; at24_emulation.c never uses its last records.
 *
 *    0x0000 - 0x33FF   firmware
 *    0x3400 - 0x73FF   EEPROM contents, 16KB, stored as-is
 *    0x7400 - 0x77FF   OLED vendor boot image (128x64, 1 bit per pixel)
 *    0x7800 - 0x7BFF   EEPROM flush scratch sector
 *    0x7C00 - 0x7FFF   EEPROM flush log sector
 *
 *  FW_LAYOUT_64K: 32KB EEPROM, L1306 only.  Compile with -DFW_LAYOUT_64K and
 *  link with -Wl,--define=FW_LAYOUT_64K (linker.cmd has the same switch).
 *
 *    0x0000 - 0x3FFF   firmware
 *    0x4000 - 0x6FFF   unused
 *    0x7000 - 0x73FF   EEPROM flush scratch sector
 *    0x7400 - 0x77FF   EEPROM flush log sector
 *    0x7800 - 0x7BFF   OLED vendor boot image
 *    0x7C00 - 0xFBFF   EEPROM contents, 32KB, stored as-is
 *    0xFC00 - 0xFFFF   unused (top sector, see above)
 *
 *  A fw_layout_t describing this is linked at FW_LAYOUT_ADDR so the flasher
 *  can find out from the firmware image where to put the EEPROM contents and
 *  the vendor image.  Firmware without it (magic missing) uses the older TI
 *  EEPROM "Type B" layout at 0x4000 with the vendor image at 0xF000.
 *
 *  Keep source/board/linker.cmd in step with these addresses.
 */

#ifndef BOARD_FW_LAYOUT_H_
#define BOARD_FW_LAYOUT_H_

#include <stdint.h>

#define FW_FLASH_SECTOR_SIZE        1024

#define FW_OLED_IMAGE_SIZE          0x00000400

#ifdef FW_LAYOUT_64K
#define FW_EEPROM_SCRATCH_ADDR      0x00007000
#define FW_EEPROM_LOG_ADDR          0x00007400
#define FW_OLED_IMAGE_ADDR          0x00007800
#define FW_EEPROM_DATA_ADDR         0x00007C00
#define FW_EEPROM_DATA_SIZE         0x00008000
#else
#define FW_EEPROM_DATA_ADDR         0x00003400
#define FW_EEPROM_DATA_SIZE         0x00004000
#define FW_OLED_IMAGE_ADDR          0x00007400
#define FW_EEPROM_SCRATCH_ADDR      0x00007800
#define FW_EEPROM_LOG_ADDR          0x00007C00
#endif

/* Just past the Cortex-M0+ vector table */
#define FW_LAYOUT_ADDR              0x000000C0
#define FW_LAYOUT_MAGIC             0x4C464B4C      /* "LKFL" in memory order */
#define FW_LAYOUT_VERSION           1

/* EEPROM contents format */
#define FW_EEPROM_FORMAT_RAW        1               /* byte N at eepromAddr + N */

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;            /* sizeof(fw_layout_t) */
    uint32_t eepromFormat;
    uint32_t eepromAddr;
    uint32_t eepromSize;
    uint32_t oledImageAddr;
    uint32_t oledImageSize;
} fw_layout_t;

#endif /* BOARD_FW_LAYOUT_H_ */
