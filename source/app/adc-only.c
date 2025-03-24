

#include "../board/ti_msp_dl_config.h"
#include "communication/i2c_tar_driver.h"
#include "emulation/at24_emulation.h"
#include "emulation/ad7291_emulation.h"
#include "system.h"


int setTargetOwnAddressAlternate() {
    return 0;
}
int setTargetOwnAddress() {
    return TAR_I2C_TARGET_OWN_ADDR + 1;
}

void setupAppSpecific() {

}