
#include "../board/ti_msp_dl_config.h"
#include "communication/i2c_tar_driver.h"
#include "emulation/at24_emulation.h"
#include "emulation/ad7291_emulation.h"
#include "system.h"


#define SSD1306_CNTRL_CMD                                          0x00
#define SSD1306_CNTRL_DATA                                         0x40

#define LED_DISPLAY_WIDTH 128
#define LED_DISPLAY_HEIGHT 64

#define SSD1306_TARGET 0x3c

/* SSD1306 Commands */
#define SSD1306_DISPLAY_OFF                                        0xAE
#define SSD1306_SET_DISP_CLK                                       0xD5
#define SSD1306_SET_MULTIPLEX                                      0xA8
#define SSD1306_SET_DISP_OFFSET                                    0xD3
#define SSD1306_SET_DISP_START_LINE                                0x40// | 0x00)
#define SSD1306_CONFIG_CHARGE_PUMP                                 0x8D
#define SSD1306_SET_MEM_ADDR_MODE                                  0x20
#define SSD1306_SEG_REMAP                                          (0xA0 | 0x01)               //Rotate 180 Degrees
#define SSD1306_SET_COMSCANDEC                                     0xC8
#define SSD1306_SET_COMPINS                                        0xDA
#define SSD1306_SET_CONTRAST                                       0x81
#define SSD1306_SET_PRECHARGE                                      0xD9
#define SSD1306_SET_VCOMDETECT                                     0xDB
#define SSD1306_DISPLAYALLON_RESUME                                0xA4
#define SSD1306_NORMAL_DISPLAY                                     0xA6
#define SSD1306_DISPLAYON                                          0xAF
#define SSD1306_SET_COL_ADDR                                       0x21
#define SSD1306_PAGEADDR                                           0x22
#define SSD1306_INVERT_DISPLAY                                     0x01
#define SSD1306_NORMALIZE_DISPLAY                                  0x00
/* SDD1306 Scroll Commands */
#define SSD1306_SET_VERTICAL_SCROLL_AREA                           0xA3
#define SSD1306_ACTIVATE_SCROLL                                    0x2F
#define SSD1306_DEACTIVATE_SCROLL                                  0x2E
#define SSD1306_RIGHT_HORIZONTAL_SCROLL                            0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL                             0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL               0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL                0x2A
#define SSD1306_INVERTDISPLAY                                      0xA7
/* SSD1306 Configuration Commands */
#define SSD1306_DISPCLK_DIV                                        0x80
#define SSD1306_MULT_64                                            0x3F
#define SSD1306_DISP_OFFSET_VAL                                    0x00
#define SSD1306_COL_START_ADDR                                     0x00                          //Reset to = 0
#define SSD1306_COL_END_ADDR                                       (LED_DISPLAY_WIDTH - 1)        //Reset to = 127
#define SSD1306_PG_START_ADDR                                      0x00
#define SSD1306_PG_END_ADDR                                        7
#define SSD1306_CHARGE_PUMP_EN                                     0x14
#define SSD1306_CONFIG_COM_PINS_64                                 0x12
#define SSD1306_CONFIG_COM_PINS_32                                 0x02
//Enable COM left to right Re-map
#define SSD1306_CONTRAST_VAL                                       0xCF                         //207
#define SSD1306_PRECHARGE_VAL                                      0xF1
#define SSD1306_VCOMH_VAL                                          0x40
#define SSD1306_MULT_DAT                                           (LED_DISPLAY_HEIGHT - 1)
#define SSD1306_HOR_MM                                             0x00

#define LED_DISPLAY_HEIGHT 64

extern i2c_tar_driver_t tar_i2c;

bool OLED_writeRegister(uint8_t reg, uint8_t v)  {
    uint8_t gTxPacket[2] = { reg, v };
    DL_I2C_fillControllerTXFIFO(tar_i2c.pModule, &gTxPacket[0], 2);

    /* Wait for I2C to be Idle */
    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;
    DL_I2C_startControllerTransfer(tar_i2c.pModule, SSD1306_TARGET, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* Poll until the Controller writes all bytes */
    while (DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) ;
    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE)) ;

    // check for error
    if (DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return true;
    }

    delay_cycles(1000);
    return false;
}
void OLED_multiWrite(uint8_t sz, uint8_t *c) {

    uint16_t idx = DL_I2C_fillControllerTXFIFO(tar_i2c.pModule, &c[0], sz);
    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    DL_I2C_startControllerTransfer(tar_i2c.pModule, SSD1306_TARGET, DL_I2C_CONTROLLER_DIRECTION_TX, sz);
    while (idx < sz) {
        /*
        //Waiting for txfifo empty
        while (!(DL_I2C_getRawInterruptStatus(tar_i2c.pModule, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY))) {

        }
        //clear interrupt
        DL_I2C_clearInterruptStatus(tar_i2c.pModule, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY);
        */
        //Write more data to I2C FIFO
        idx += DL_I2C_fillControllerTXFIFO(tar_i2c.pModule, &c[idx], sz - idx);
    }
    while (DL_I2C_getControllerStatus(tar_i2c.pModule) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;

    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    delay_cycles(1000);
}

#define MAX_CHUNK_SIZE 129
uint8_t chunk[MAX_CHUNK_SIZE] = {0};
#define SCREEN_SIZE (LED_DISPLAY_WIDTH*LED_DISPLAY_HEIGHT/8)+MAX_CHUNK_SIZE

const uint8_t *oledScreen = (const uint8_t *)0x0000F000;

inline void clearMem(uint8_t *b, uint16_t sz) {
    for (int x = 0; x < sz; x++) {
        b[x] = 0;
    }
}

void DisplayScreen() {
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_COL_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_COL_START_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_COL_END_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PAGEADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PG_START_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PG_END_ADDR);

    short loop_1 = 0, loop_2 = 0;
    short index = 0x00;
    int max = LED_DISPLAY_HEIGHT * LED_DISPLAY_WIDTH / 8;
    for (loop_1 = 0; loop_1 < 1024; loop_1++) {
        chunk[0] = 0x40;
        for(loop_2 = 1; loop_2 < MAX_CHUNK_SIZE; loop_2++) {
            chunk[loop_2] = oledScreen[index++];
        }
        
        OLED_multiWrite(MAX_CHUNK_SIZE, chunk);
        clearMem(chunk, MAX_CHUNK_SIZE);
        
        if (index >= max)
            break;
    }
}

void doOLED() {
    if (OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPLAY_OFF)) {
        return;
    }
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_MULTIPLEX);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_MULT_DAT);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_DISP_CLK);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPCLK_DIV);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_DISP_OFFSET);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISP_OFFSET_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_DISP_START_LINE);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CONFIG_CHARGE_PUMP);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CHARGE_PUMP_EN);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_MEM_ADDR_MODE);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_HOR_MM);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SEG_REMAP);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_COMSCANDEC);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_COMPINS);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CONFIG_COM_PINS_64);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_PRECHARGE);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PRECHARGE_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_VCOMDETECT);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_VCOMH_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPLAYALLON_RESUME);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_NORMAL_DISPLAY);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_CONTRAST);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CONTRAST_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DEACTIVATE_SCROLL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPLAYON);
    DisplayScreen();
}

static const DL_I2C_ClockConfig gTAR_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};
void System_InitOLED() {
    DL_I2C_disablePower(tar_i2c.pModule);
    DL_I2C_reset(tar_i2c.pModule);
    DL_I2C_enablePower(tar_i2c.pModule);

    DL_I2C_setClockConfig(tar_i2c.pModule, (DL_I2C_ClockConfig *) &gTAR_I2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(tar_i2c.pModule);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(tar_i2c.pModule);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(tar_i2c.pModule, 7);
    DL_I2C_setControllerTXFIFOThreshold(tar_i2c.pModule, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(tar_i2c.pModule, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(tar_i2c.pModule);
    /* Enable module */
    DL_I2C_enableController(tar_i2c.pModule);

    doOLED();

    DL_I2C_disableController(tar_i2c.pModule);

    DL_I2C_disablePower(tar_i2c.pModule);
    DL_I2C_reset(tar_i2c.pModule);
    DL_I2C_enablePower(tar_i2c.pModule);
    DL_I2C_setClockConfig(TAR_I2C_INST,
        (DL_I2C_ClockConfig *) &gTAR_I2CClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(TAR_I2C_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(TAR_I2C_INST);

    /* Configure Target Mode */
    DL_I2C_setTargetOwnAddress(TAR_I2C_INST, TAR_I2C_TARGET_OWN_ADDR);
    DL_I2C_setTargetOwnAddressAlternate(TAR_I2C_INST, TAR_I2C_TARGET_SEC_OWN_ADDR);
    DL_I2C_enableTargetOwnAddressAlternate(TAR_I2C_INST);
    DL_I2C_setTargetTXFIFOThreshold(TAR_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setTargetRXFIFOThreshold(TAR_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableTargetTXEmptyOnTXRequest(TAR_I2C_INST);

    DL_I2C_enableTargetClockStretching(TAR_I2C_INST);
}

int setTargetOwnAddressAlternate() {
    return TAR_I2C_TARGET_SEC_OWN_ADDR;
}
int setTargetOwnAddress() {
    return TAR_I2C_TARGET_OWN_ADDR;
}

void setupAppSpecific() {
    System_InitOLED();
}