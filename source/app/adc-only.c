

#include "../board/ti_msp_dl_config.h"
#include "communication/i2c_tar_driver.h"
#include "emulation/at24_emulation.h"
#ifdef ADC_AD7998
#include "emulation/ad7998_emulation.h"
#else
#include "emulation/ad7291_emulation.h"
#endif
#include "system.h"


int setTargetOwnAddressAlternate() {
    return 0;
}
int setTargetOwnAddress() {
    return TAR_I2C_TARGET_OWN_ADDR + 1;
}

void setupAppSpecific() {

}

void openADCEmulation(i2c_tar_driver_t *pI2CTarDriverInst) {
#ifdef ADC_AD7998
    pI2CTarDriverInst->rxCallback = &ad7998_i2c_rx_callback;
    pI2CTarDriverInst->txCallback = &ad7998_i2c_tx_callback;
    ad7998_open(pI2CTarDriverInst);
#else
    pI2CTarDriverInst->rxCallback = &ad7291_i2c_rx_callback;
    pI2CTarDriverInst->txCallback = &ad7291_i2c_tx_callback;
    ad7291_open(pI2CTarDriverInst);
#endif
}
