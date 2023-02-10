/*
 * spi_target.c
 *
 *  Created on: Oct 27, 2022
 *      Author: munan
 */

#include <spi_target.h>

volatile uint8_t spiFlag;

#define MIN_REG_VALUE       0
#define MAX_REG_VALUE       26

#define MAX_REG_SIZE        MAX_REG_VALUE-MIN_REG_VALUE


uint8_t regPointer = 0;
uint8_t gRxCount = 0;

uint8_t gTest = 0;
uint8_t gRxPacket[MAX_REG_SIZE];

typedef enum SPI_deviceState{
    RX_CMD,
    RX_DATA,
    TX_DATA,
    IDLE,
}SPI_deviceState;

volatile SPI_deviceState gSpiState = IDLE;

void GROUP1_IRQHandler(void){
    switch(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)){
    case GPIO_GRP_0_INT_IIDX:
        if(DL_GPIO_readPins(GPIOB, GPIO_GRP_0_SPI_CS_PIN)){   //if CS toggles High: Put SPI in Idle Mode
            gSpiState = IDLE;
        }
        else{                                                           //of CS toggles Low: Reset SPI transaction State to ready
            spi_reset();
        }
        break;
    default:
        break;
    }
}

void SPI_target_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_target_INST)) {
        case DL_SPI_IIDX_RX:
            switch(gSpiState){
                case(RX_CMD):
                    regPointer = DL_SPI_receiveData8(SPI_target_INST);
                    if((regPointer & 0x80)){
                        gSpiState = RX_DATA;
                        gRxCount = 0;
                        regPointer = regPointer - 0x80;
                    }
                    else{
                        spi_targetTx(regPointer);
                        gSpiState = TX_DATA;
                        regPointer++;
                    }
                    break;
                case(TX_DATA):
                    DL_SPI_receiveData8(SPI_target_INST);  //throw out received data its garbage
                    if(regPointer < MAX_REG_VALUE){
                        spi_targetTx(regPointer);
                        regPointer++;
                    }
                    else{
                        while(DL_SPI_transmitDataCheck8(SPI_target_INST, 0xAA) != false);
                    }
                    break;
                case(RX_DATA):
                    if(gRxCount < MAX_REG_SIZE){
                        spi_targetRx(regPointer++);

                    }
                    else{
                        DL_SPI_receiveData8(SPI_target_INST);
                    }
                    break;
                case(IDLE):     //chip select pin is not active
                    DL_SPI_receiveData8(SPI_target_INST);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

inline void spi_reset(void){
    gSpiState = RX_CMD;
    spiFlag = 0;
    gRxCount = 0;
    gTest++;
}

inline void spi_targetTx(uint8_t regPointer)
{
    switch(regPointer)
    {
        case 1: /* WakeInt */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getWakeInt());
            break;

        case 6: /* OutCFG */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getOutCfg());
            break;

        case 7: /* Seconds */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getSeconds());
            break;

        case 8: /* Minutes */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getMinutes());
            break;

        case 9: /* Hours */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getHours());
            break;

        case 10: /* Day of Week */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getWeekday());
            break;

        case 11: /* Date */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getDate());
            break;

        case 12: /* Month */
            DL_SPI_transmitData8(SPI_target_INST, rtc_getMonth());
            break;

        case 13: /* High Byte of Year */
            DL_SPI_transmitData8(SPI_target_INST, (uint8_t)((rtc_getYear() >> 8) & 0xFF)); //high byte of year data
            break;

        case 14: /* Low Byte of Year */
            DL_SPI_transmitData8(SPI_target_INST, (uint8_t)(rtc_getYear() & 0x00FF));      //low byte of year data
            break;

        default:
            DL_SPI_transmitData8(SPI_target_INST, 0xAA); //Send dummy byte if not a valid address
            break;
    }

}

inline void spi_targetRx(uint8_t regPointer)
{
    switch(regPointer)
    {
        case 1: /* WakeInt */
            rtc_setWakeInt(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 6: /* OutCFG */
            rtc_setOutCfg(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 7: /* Seconds */
            rtc_setSeconds(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 8: /* Minutes */
            rtc_setMinutes(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 9: /* Hours */
            rtc_setHours(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 10: /* Day of Week */
            rtc_setWeekday(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 11: /* Date */
            rtc_setDate(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 12: /* Month */
            rtc_setMonth(DL_SPI_receiveData8(SPI_target_INST));
            break;

        case 13: /* High Byte of Year */
            rtc_setYear(((uint16_t)(DL_SPI_receiveData8(SPI_target_INST) << 8)) + (rtc_getYear() & 0x00FF));
            break;

        case 14: /* Low Byte of Year */
            rtc_setYear(((uint16_t) DL_SPI_receiveData8(SPI_target_INST)) + (rtc_getYear() & 0xFF00));
            break;

        default:
            DL_SPI_receiveData8(SPI_target_INST);
            break;  // Just throw away the byte if not a valid address

    }
}
