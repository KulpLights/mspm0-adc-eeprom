/*
 * i2c_target.c
 *
 *  Created on: Oct 27, 2022
 *      Author: munan
 */

#include "i2c_target.h"

#define MIN_REG_VALUE       0
#define MAX_REG_VALUE       26

#define MAX_REG_SIZE        MAX_REG_VALUE-MIN_REG_VALUE


static uint8_t regPointer = 0;
uint8_t gRxCount = 0;
uint8_t gRxPacket[MAX_REG_SIZE];
uint8_t gTxPacket;


void I2C_target_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_target_INST)) {
        case DL_I2C_IIDX_TARGET_START:
            gRxCount = 0;                                    //reset the input count
            DL_I2C_flushTargetTXFIFO(I2C_target_INST);      //flush the TX buffer to make sure fresh read data is sent
            break;
        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            /* Controller write - Store received data */
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_target_INST) != true){
                if (gRxCount < MAX_REG_SIZE){
                    gRxPacket[gRxCount++] = DL_I2C_receiveTargetData(I2C_target_INST);
                }
                else{
                    DL_I2C_receiveTargetData(I2C_target_INST);      //discard extra inputed Data
                }
            }
            regPointer = gRxPacket[0];                              //first instance: received reg address for either read or write
            break;
        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:            //triggered when TXfifo is empty (byte is sent on line or when start is received)
            if(regPointer < MAX_REG_VALUE){
                i2c_targetTx(regPointer);
                regPointer++;               //increment the register pointer for multibyte read
                DL_I2C_transmitTargetDataCheck(I2C_target_INST, gTxPacket);
            }
            else{                           //controller is asking for data outside of reg map: send garbage
                while(DL_I2C_transmitTargetDataCheck(I2C_target_INST, 0xAA) != false);
            }
            break;
        case DL_I2C_IIDX_TARGET_STOP:
            DL_I2C_flushTargetTXFIFO(I2C_target_INST); /*Flush TX FIFO incase controller didn't read last byte*/
            //after all the data is read from the controller, update the registers
            for(int i = 1; i < gRxCount; i++){
                i2c_targetRx(regPointer, gRxPacket[i]);
                regPointer++;
            }
            break;

        default:
            break;
    }
}


inline void i2c_targetTx(uint8_t regPointer)
{
    switch(regPointer)
    {
        case 1: /* WakeInt */
            gTxPacket = rtc_getWakeInt();
            break;

        case 6: /* OutCFG */
            gTxPacket = rtc_getOutCfg();
            break;

        case 7: /* Seconds */
            gTxPacket = rtc_getSeconds();
            break;

        case 8: /* Minutes */
            gTxPacket = rtc_getMinutes();
            break;

        case 9: /* Hours */
            gTxPacket = rtc_getHours();
            break;

        case 10: /* Day of Week */
            gTxPacket = rtc_getWeekday();
            break;

        case 11: /* Date */
            gTxPacket = rtc_getDate();
            break;

        case 12: /* Month */
            gTxPacket = rtc_getMonth();
            break;

        case 13: /* High Byte of Year */
            gTxPacket = (uint8_t)((rtc_getYear() >> 8) & 0xFF); //high byte of year data
            break;

        case 14: /* Low Byte of Year */
            gTxPacket = (uint8_t)(rtc_getYear() & 0x00FF);      //low byte of year data
            break;
        default:
            DL_I2C_transmitTargetData(I2C_target_INST, 0xAA); //Send dummy byte if not a valid address
            break;
    }

}

inline void i2c_targetRx(uint8_t regPointer, uint8_t inputData)
{
    switch(regPointer)
    {
        case 1: /* WakeInt */
            rtc_setWakeInt(inputData);
            break;

        case 6: /* OutCFG */
            rtc_setOutCfg(inputData);
            break;

        case 7: /* Seconds */
            rtc_setSeconds(inputData);
            break;

        case 8: /* Minutes */
            rtc_setMinutes(inputData);
            break;

        case 9: /* Hours */
            rtc_setHours(inputData);
            break;

        case 10: /* Day of Week */
            rtc_setWeekday(inputData);
            break;

        case 11: /* Date */
            rtc_setDate(inputData);
            break;

        case 12: /* Month */
            rtc_setMonth(inputData);
            break;

        case 13: /* High Byte of Year */
            rtc_setYear(((uint16_t)(inputData << 8)) | (rtc_getYear() & 0x00FF));
            break;

        case 14: /* Low Byte of Year */
            rtc_setYear(((uint16_t) inputData)| (rtc_getYear() & 0xFF00));
            break;

        default:
            break;  // Just throw away the byte if not a valid address

    }
}
