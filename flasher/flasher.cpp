#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include <jsoncpp/json/json.h>

#include <util/GPIOUtils.h>
#include "util/BBBUtils.h"
#include "util/I2CUtils.h"
#include "common.h"


constexpr uint8_t BSL_HEADER = 0x80;
constexpr uint8_t CMD_CONNECTION = 0x12;
constexpr uint8_t CMD_GET_ID = 0x19;
constexpr uint8_t CMD_PASSWORD = 0x21;
constexpr uint8_t CMD_MASS_ERASE = 0x15;
constexpr uint8_t CMD_PROGRAM = 0x20;
constexpr uint8_t CMD_READ = 0x29;
constexpr uint8_t CMD_START_APP = 0x40;

static const std::vector<uint8_t> PASSWORD_PACK = {
    BSL_HEADER, 33, 0x00, CMD_PASSWORD, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static const std::vector<uint8_t> CONNECTION_PACK = {
    BSL_HEADER, 0x01, 0x00, CMD_CONNECTION
};
static const std::vector<uint8_t> ID_PACK = {
    BSL_HEADER, 0x01, 0x00, CMD_GET_ID
};
static const std::vector<uint8_t> MASS_ERASE_PACK = {
    BSL_HEADER, 0x01, 0x00, CMD_MASS_ERASE
};
static const std::vector<uint8_t> PROGRAM_PACK_HEADER = {
    BSL_HEADER, 0x00, 0x00, CMD_PROGRAM
};

static const std::vector<uint8_t> SUCCESS_RESPONSE = {
    0x00, 0x08, 0x02, 0x00, 0x3b, 0x00, 0x38, 0x02, 0x94, 0x82
};

static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    uint32_t crc32_POLY = 0xEDB88320;
    for (size_t i = 0; i < len; i++) {
        crc = crc ^ data[i];
        for (int ii = 1; ii <= 8; ii++) {
            int mask = -(crc & 1);
            crc = (crc >> 1) ^ (crc32_POLY & mask);
        }
    }
    return crc;
}

void sendPack(I2CUtils &i2c, const std::vector<uint8_t> &pack) {
    uint32_t checksum = crc32(&pack[3], pack.size() - 3);
    uint8_t *bsptr = (uint8_t*)&checksum;
    std::vector<uint8_t> data = pack;
    data.insert(data.end(), bsptr, bsptr + 4);
    printf("Sending ----\n");
    for (int x = 0; x < data.size(); x++) {
        printf("%02X", data[x]);
    }
    printf("\n");

    i2c.writeDevice(&data[0], data.size());
}
bool checkResponse(I2CUtils &i2c) {
    bool ret = true;
    for (int x = 0; x < SUCCESS_RESPONSE.size(); x++) {
        uint8_t r = i2c.readByteData(0);
        if (r != SUCCESS_RESPONSE[x]) {
            printf("Failed at %d    %02x/%02x\n", x, r, SUCCESS_RESPONSE[x]);
            ret = false;
        }
    }
    return ret;
}
std::vector<uint8_t> hexStringToBytes(const std::string &hex) {
    std::vector<std::string> hexes = split(hex, ' ');
    std::vector<uint8_t> bytes;
    for (auto &h : hexes) {
        TrimWhiteSpace(h);
        bytes.push_back(strtol(h.c_str(), NULL, 16));
    }
    return bytes;
}
bool packAndFlash(I2CUtils &i2c, uint32_t address, const std::vector<uint8_t> &curData) {
    std::vector<uint8_t> pack = PROGRAM_PACK_HEADER;
    uint8_t *aptr = (uint8_t*)&address;
    int pos = 0;
    while (pos < curData.size()) {
        pack = PROGRAM_PACK_HEADER;
        pack.insert(pack.end(), aptr, aptr + 4);
        int len = std::min(128, (int)curData.size() - pos);
        pack[1] = len + 5;
        pack.insert(pack.end(), curData.begin() + pos, curData.begin() + pos + len);
        sendPack(i2c, pack);
        if (!checkResponse(i2c)) {
            printf("BSL Program Failed\n");
            return false;
        }
        pos += len;
        address += len;
    }
    return true;
}
bool uploadFirmware(I2CUtils &i2c, const std::string &fname) {
    std::string data = GetFileContents(fname);
    if (data.empty()) {
        printf("Failed to open file\n");
        return false;
    }    
    std::vector<std::string> lines = split(data, '\n');
    std::vector<uint8_t> curData;
    uint32_t address = 0;
    for (auto line : lines) {
        TrimWhiteSpace(line);
        if (line.empty()) {
            continue;
        }
        if (line == "q" || line[0] == '@') {
            if (!curData.empty()) {
                if (!packAndFlash(i2c, address, curData)) {
                    return false;
                }
                curData.clear();
            }
            if (line == "q") {
                return true;
            }
            address = strtol(line.substr(1).c_str(), NULL, 16);
        } else {
            std::vector<uint8_t> bytes = hexStringToBytes(line);
            if (bytes.empty()) {
                printf("Invalid hex string\n");
                return false;
            }
            curData.insert(curData.end(), bytes.begin(), bytes.end());
        }        
    }
    if (!curData.empty()) {
        if (!packAndFlash(i2c, address, curData)) {
            return false;
        }
    }
    return true;
}
bool uploadEEProm(I2CUtils &i2c, const std::string &fname) {
    std::vector<uint8_t> bytes = {0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
    int file = open(fname.c_str(), O_RDONLY);
    if (file < 0) {
        printf("Failed to open eeprom file\n");
        return false;
    }
    uint8_t buffer[8192];
    memset(buffer, 0xFF, 8192);
    int len = read(file, buffer, 8192);
    if (len < 0) {
        printf("Failed to read eeprom file\n");
        return false;
    }
    bytes.reserve(bytes.size() + len * 2);
    uint32_t index = 0;
    uint8_t *iptr = (uint8_t*)&index;
    for (int x = 0; x < len; x += 4) {
        bytes.insert(bytes.end(), iptr, iptr + 4);
        index++;
        bytes.push_back(buffer[x]);
        bytes.push_back(buffer[x + 1]);
        bytes.push_back(buffer[x + 2]);
        bytes.push_back(buffer[x + 3]);
    }
    uint32_t address = 0x00003800;
    if (!packAndFlash(i2c, address, bytes)) {
        return false;
    }
    return true;
}
int main(int argc, char **argv) {
    PinCapabilities::InitGPIO("mp0-flasher", new BBBPinProvider());

    std::string nrst = "pca9674-0";
    std::string binvoke = "pca9674-1";
    std::string fname = "pcu_pocketbeagle2.txt";
    std::string eeprom = "eeprom.bin";

    I2CUtils i2c(2, 0x48);

    printf("\nPB2 MSP M0 Flasher\n");

    const PinCapabilities &nrstPin = PinCapabilities::getPinByName(nrst);
    const PinCapabilities &binvokePin = PinCapabilities::getPinByName(binvoke);

    if (!nrstPin.ptr() || !binvokePin.ptr()) {
        printf("Invalid pin configuration\n");
        return 1;
    }
    nrstPin.configPin("gpio", true);
    binvokePin.configPin("gpio", true);

    nrstPin.setValue(1);
    binvokePin.setValue(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    nrstPin.setValue(0);
    binvokePin.setValue(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    nrstPin.setValue(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    printf("Reset Released... M0 Entering BSL Mode...\n");
    sendPack(i2c, CONNECTION_PACK);
    if (i2c.readByte() != 0) {
        printf("BSL Connection Failed\n");
        return 1;
    } else {
        printf("BSL Connection Success\n");
    }

    sendPack(i2c, PASSWORD_PACK);
    if (!checkResponse(i2c)) {
        printf("BSL Password Failed\n");
        return 1;
    } else {
        printf("BSL Password Success\n");
    }
    sendPack(i2c, MASS_ERASE_PACK);
    if (!checkResponse(i2c)) {
        printf("BSL Mass Erase Failed\n");
        return 1;
    } else {
        printf("BSL Mass Erase Success\n");
    }

    uploadFirmware(i2c, fname);

    uploadEEProm(i2c, eeprom);

    printf("Cleaning Up...\n");
    nrstPin.setValue(0);
    binvokePin.setValue(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    printf("RESET M0...\n");
    nrstPin.setValue(1);
    binvokePin.setValue(1);
    printf("Done\n");
    return 0;
}