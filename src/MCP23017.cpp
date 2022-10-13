#include "MCP23017.h"


 MCP23017::MCP23017(uint8_t i2c_bus, uint8_t i2c_addr) {
    bus = i2c_bus;           // I2C bus of Jetson (1 and 8 available on Xavier)
    address = i2c_addr ; // Address of MCP23017; defaults to 0x20
}

MCP23017::~MCP23017() {
    closeI2C() ;
}

//helper function to replace bitRead function from Arduino library
bool MCP23017::bitRead(uint8_t num, uint8_t index) {
    return (num >> index) & 1;
}

/**
 * @brief helper function to replace bitWrite function from Arduino library
 * @param var variable
 * @param index index of the bit
 * @param bit  bit value
 */
void MCP23017::bitWrite(uint8_t &var, uint8_t index, uint8_t bit) {
    uint new_bit = 1 << index;
    if(bit) {
        var = var | new_bit;
    }
    else {
        new_bit = ~new_bit;
        var = var & new_bit;
    }
}

/**
 * @brief open I2C communication
 * @return -1 on failure, 1 on success
 */
bool MCP23017::beginI2C() {
    char device[32];
    snprintf(device, sizeof(device), "/dev/i2c-%u", bus);
    if ((fd = open(device, O_RDWR)) < 0) {
        printf("File descriptor opening error %s\n", strerror(errno));
        return -1;
    }
    else {
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            printf("File descriptor opening error %s\n", strerror(errno));
            return -1;
        }
    }
    return 1;
}

//close I2C communication
void MCP23017::closeI2C() {
    if (fd > 0) {
        close(fd);
        fd = -1 ;
    }
}

/**
 * @brief Bit number associated to a given Pin
 * @param pin corresponding pin
 * @return uint8_t mod8 of pin
 */
uint8_t MCP23017::bitForPin(uint8_t pin) {
    return pin % 8;
}

/**
 * @brief Register address, port dependent, for a given PIN
 * @param pin corresponding Pin
 * @param portA port a reg
 * @param portB port b reg
 * @return uint8_t selects respective port
 */
uint8_t MCP23017::regForPin(uint8_t pin, uint8_t portA, uint8_t portB) {
    return (pin < 8) ? portA : portB;
}

/**
 * @brief Reads a given register
 * @param reg address of the register
 * @return uint8_t value of the register
 */
int MCP23017::readRegister(uint8_t reg)
{
    int toReturn = i2c_smbus_read_byte_data(fd, reg);
    if (toReturn < 0)
    {
        printf("MCP23017 Read Byte error %s\n", strerror(errno));
        return -1;
    }
#ifdef ENABLE_DEBUG_OUTPUT
    printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", address, toReturn, readRegister);
#endif // ENABLE_DEBUG_OUTPUT
    return toReturn;
}

/**
 * @brief Writes a value to corresponding register
 * @param reg  register address
 * @param Value value to be written
 * @return -1 on failure, 1 on success
 */
int MCP23017::writeRegister(uint8_t reg, uint8_t Value) {
#ifdef ENABLE_DEBUG_OUTPUT
    printf("Wrote: 0x%02X to register 0x%02X \n", writeValue, writeRegister);
#endif
    int retVal = i2c_smbus_write_byte_data(fd, reg, Value);
    if (retVal < 0)
    {
        printf("Write I2C device failed %s\n", strerror(errno));
        return -1;
    }
    return retVal;
}

/**
 * @brief Read byte from I2C bus
 * @return uint8_t value to be read
 */
uint8_t MCP23017::readByte() {
    uint8_t retVal = i2c_smbus_read_byte(fd);
    if (retVal < 0) {
        printf("MCP23017 Read Byte error %s\n", strerror(errno));
        return -1;
    }
#ifdef ENABLE_DEBUG_OUTPUT
    printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", address, retVal, readRegister);
#endif

    return retVal;
}

/**
 * @brief  Wrtie a Byte to  I2c bus
 * @param Value value to be written
 * @return uint8_t -1 on failure, 1 on success
 */
uint8_t MCP23017::writeByte(uint8_t Value) {
#ifdef ENABLE_DEBUG_OUTPUT
    printf("Wrote: 0x%02X to register 0x%02X \n", writeValue, writeRegister);
#endif
    int retVal = i2c_smbus_write_byte(fd, Value);
    if (retVal < 0)
    {
        printf("MCP23017 Write Byte error %s\n", strerror(errno));
        return -1;
    }
    return retVal;
}

/**
 * @brief  Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 * @param pin corresponding pin
 * @param value required value
 * @param portA first 8 pins (IOBANK = 0)
 * @param portB last 8 pins (IOBANK = 0)
 */
void MCP23017::updateRegisterBit(uint8_t pin, uint8_t value, uint8_t portA, uint8_t portB) {
    uint8_t regVal;
    uint8_t reg = regForPin(pin, portA, portB);
    uint8_t bit = bitForPin(pin);
    regVal = readRegister(reg);

    // set the value for the particular bit
    bitWrite(regVal, bit, value);

    writeRegister(reg, regVal);
}

/**
 * @brief sets the pin mode to either INPUT or OUTPUT
 * @param pin  corresponding pin
 * @param mode required mode
 */
void MCP23017::pinMode(uint8_t pin, uint8_t mode) {
    updateRegisterBit(pin, (mode == MCP_INPUT), MCP23017_IODIRA, MCP23017_IODIRB);
}

/**
 * @brief Reads all 16 pins (port A and B) into a single 16 bits variable.
 * @return uint16_t required read value
 */
uint16_t MCP23017::readGPIOAB() {
    uint8_t buffer[2];
    writeByte(MCP23017_GPIOA);
    buffer[0] = readByte();
    buffer[1] = readByte();
    return ((buffer[1] << 8) | buffer[0]);
}

/**
 * @brief  Read a single port, A or B, and return its current 8 bit value.
 * @param pin corresponding pin
 * @return uint8_t gpio read value
 */
uint8_t MCP23017::readGPIO(uint8_t pin) {
    // read the current GPIO output latches
    if (pin == 0) {
        writeByte(MCP23017_GPIOA);
    }
    else {
        writeByte(MCP23017_GPIOB);
    }
    uint8_t value = readByte();
    return value;
}

/**
 * @brief Writes all the pins in one go
 * This method is very useful if you are implementing a multiplexed matrix
 * @param value value to be written
 */
void MCP23017::writeGPIOAB(uint16_t value) {
    writeByte(MCP23017_GPIOA);
    writeByte(lowByte(value));
    writeByte(highByte(value));
}

/**
 * @brief write value to GPIO, (HIGH, LOW)
 * @param pin corresponding pin
 * @param value required value
 */
void MCP23017::digitalWrite(uint8_t pin, uint8_t value) {
    uint8_t gpio;
    uint8_t bit = bitForPin(pin);
    // read the current GPIO output latches
    uint8_t reg = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
    gpio = readRegister(reg);
    // set the pin and direction
    bitWrite(gpio, bit, value);
    // write the new GPIO
    reg = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    writeRegister(reg, gpio);
}

/**
 * @brief this method is used to pullup gpio
 * @param pin corresponding pin
 * @param value required value
 */
void MCP23017::pullUp(uint8_t pin, uint8_t value) {
    updateRegisterBit(pin, value, MCP23017_GPPUA, MCP23017_GPPUB);
}

/**
 * @brief the function return the status of the pin, 0, 1
 * @param pin pin number
 * @return return 1, if the volatages on pin are morethan threshold voltages
 *         otherwise returns zero
 */
bool MCP23017::digitalRead(uint8_t pin) {
    uint8_t bit = bitForPin(pin);
    uint8_t reg = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    return (readRegister(reg) >> bit) & 0x1;
}

/**
 * @brief Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false, false, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 * @param mirroring Sets mirroring bit
 * @param openDrain Sets open drain bit
 * @param polarity  Sets
 */
void MCP23017::setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity) {
    // configure the port A
    uint8_t ioconfValue = readRegister(MCP23017_IOCONA);
    bitWrite(ioconfValue, 6, mirroring);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    writeRegister(MCP23017_IOCONA, ioconfValue);
    // Configure the port B
    ioconfValue = readRegister(MCP23017_IOCONB);
    bitWrite(ioconfValue, 6, mirroring);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    writeRegister(MCP23017_IOCONB, ioconfValue);
}

/**
 * @brief  Set's up a pin for interrupt. uses three MODEs: CHANGE, FALLING, RISING.
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 * @param pin corresponding pin
 * @param mode required mode: CHANGE, FALLING, RISING
 */
void MCP23017::setupInterruptPin(uint8_t pin, uint8_t mode) {
    // set the pin interrupt control (0 means change, 1 means compare against given value);
    updateRegisterBit(pin, (mode != CHANGE), MCP23017_INTCONA, MCP23017_INTCONB);
    // if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt
    // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
    // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
    updateRegisterBit(pin, (mode == FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB);
    // enable the pin for interrupt
    updateRegisterBit(pin, HIGH, MCP23017_GPINTENA, MCP23017_GPINTENB);
}

uint8_t MCP23017::getLastInterruptPin() {
    uint8_t intf;
    // try port A
    intf = readRegister(MCP23017_INTFA);
    for (int i = 0; i < 8; i++) {
        if (bitRead(intf, i))
            return i;
    }
    // try port B
    intf = readRegister(MCP23017_INTFB);
    for (int i = 0; i < 8; i++) {
        if (bitRead(intf, i))
            return i + 8;
    }
    return MCP23017_INT_ERR;
}

uint8_t MCP23017::getLastInterruptPinValue() {
    uint8_t intPin = getLastInterruptPin();
    if (intPin != MCP23017_INT_ERR)
    {
        uint8_t intcapreg = regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
        uint8_t bit = bitForPin(intPin);
        return (readRegister(intcapreg) >> bit) & (0x01);
    }

    return MCP23017_INT_ERR;
}
