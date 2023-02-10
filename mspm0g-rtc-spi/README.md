## Example Summary

Base project for MSPM0 based RTC and ADC firmware

## Peripherals & Pin Assignments

### RTC Pin Assignments
| IO | Package Pin Number | Function |
|----|--------------------|----------|
| PA3 | 43 | LFXTIN |
| PA4 | 44 | LFXTOUT | 

### I2C Pin Assignments
| IO | Package Pin Number | Function |
|----|--------------------|----------|
| PB2 | 50 | SCL - I2C1 |
| PB3 | 51 | SDA - I2C1 | 

#### I2C Communication
The I2C communication model of this device is similar to that of TI BQ32002. I2C communication is initiated by a controller sending a start condition, a high-to-low transition on the SDA I/O while SCL is held high. After the start
condition, the device address byte is sent, most-significant bit (MSB) first, including the data direction bit (R/W). After receiving a valid address byte, this device responds with an
acknowledge, a low on the SDA I/O during the high of the acknowledge-related clock pulse. This device responds to the I2C target address ????????b.

This device does not respond to the general call address.

One or more data bytes follow the address acknowledge. 
- If the R/W bit is low, the data is written from the controller. The first byte written initializes a subaddress register that points to the device register to be read or written in subsequent transations. 
- If the R/W bit is high, the data from this device are the values read from the register previously selected by a write to the subaddress register. The data byte is followed by an acknowledge sent from this device. Data is output only if complete bytes are received and acknowledged.
A stop condition, which is a low-to-high transition on the SDA I/O while the SCL input is high, is sent by the
controller to terminate the transfer. 

#### I2C Write
| START | Device Address | W |  Subaddress  | RegN write | RegN+1 write | ...| STOP |
|-------|----------------|---|--------------|------------|--------------|----|------|

#### I2C Read
| START | Device Address | W |  Subaddress  | STOP | START | Device Address | R |  RegN Read  |  RegN+1 Read | ...| STOP |
|-------|----------------|---|--------------|------|-------|----------------|---|-------------|--------------|----|------|


### JTAG Pin Assignments
| IO | Package Pin Number | Function |
|----|--------------------|----------|
| PA19 | 12 | SWDIO |
| PA20 | 13 | SWCLK | 

## BoosterPacks, Board Resources & Jumper Settings

- N/A

## Example Usage

Compile, load and run the example.
