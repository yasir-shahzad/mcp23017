#ifndef _MCP23017_H
#define _MCP23017_H

#include <cstddef>
#include <cstdint>

extern "C" {
#include <i2c/smbus.h>
}

#include <cstdio>
#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>


//#define ENABLE_DEBUG_OUTPUT

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define MCP23017_ADDRESS                           (0x20) ///< 0100 001 (CONTRL REG)
/*=========================================================================*/

// registers
//Port A (IOBANK = 1)
#define MCP23017_IODIRA        (0x00)  //!< I/O direction register
#define MCP23017_IPOLA         (0x02)  //!< Input polarity register
#define MCP23017_GPINTENA      (0x04)  //!< Interrupt-on-change control register 
#define MCP23017_DEFVALA       (0x06)  //!< Default compare register for interrupt-on-change
#define MCP23017_INTCONA       (0x08)  //!< Interrupt control register
#define MCP23017_IOCONA        (0x0A)  //!< Configuration register
#define MCP23017_GPPUA         (0x0C)  //!< Pull-up resistor configuration register
#define MCP23017_INTFA         (0x0E)  //!< Interrupt flag register
#define MCP23017_INTCAPA       (0x10)  //!< Interrupt capture register
#define MCP23017_GPIOA         (0x12)  //!< Port register
#define MCP23017_OLATA         (0x14)  //!< Output latch register
//Port B (IOBANK = 0)
#define MCP23017_IODIRB        (0x01)  //!< I/O direction register
#define MCP23017_IPOLB         (0x03)  //!< Input polarity register
#define MCP23017_GPINTENB      (0x05)  //!< Interrupt-on-change control register
#define MCP23017_DEFVALB       (0x07)  //!< Default compare register for interrupt-on-change
#define MCP23017_INTCONB       (0x09)  //!< Interrupt control register
#define MCP23017_IOCONB        (0x0B)  //!< Configuration register
#define MCP23017_GPPUB         (0x0D)  //!< Pull-up resistor configuration register
#define MCP23017_INTFB         (0x0F)  //!< Interrupt flag register
#define MCP23017_INTCAPB       (0x11)  //!< Interrupt capture register
#define MCP23017_GPIOB         (0x13)  //!< Port register
#define MCP23017_OLATB         (0x15)  //!< Output latch register

#define MCP23017_INT_ERR 255

//constants set up to emulate Arduino pin parameters
#define HIGH 1
#define LOW  0

#define CHANGE  0
#define FALLING 1
#define RISING  2

#define MCP_INPUT  1
#define MCP_OUTPUT 0

//macros
#define lowByte(w) ((uint8_t) ((w) & 0xFF))
#define highByte(w) ((uint8_t) ((w) >> 8))


class MCP23017
{
  public:
    int fd;          // File Descriptor to the MCP23017
    uint8_t bus;     // I2C bus of the MCP23017
    uint8_t address; // Address of MCP23017; defaults to 0x20
    MCP23017(uint8_t i2c_bus = 1, uint8_t i2c_addr = MCP23017_ADDRESS);
    ~MCP23017();

    bool beginI2C();
    void closeI2C();
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    void pullUp(uint8_t pin, uint8_t value);
    bool digitalRead(uint8_t pin);
    void writeGPIOAB(uint16_t value);
    uint16_t readGPIOAB();
    uint8_t readGPIO(uint8_t pin);
    void setupInterrupts(uint8_t mirroring, uint8_t open, uint8_t polarity);
    void setupInterruptPin(uint8_t p, uint8_t mode);
    uint8_t getLastInterruptPin();
    uint8_t getLastInterruptPinValue();

  private:
    uint8_t bitForPin(uint8_t pin);
    uint8_t regForPin(uint8_t pin, uint8_t portA, uint8_t portB);
    int readRegister(uint8_t reg);
    int writeRegister(uint8_t reg, uint8_t value);
    uint8_t readByte();
    uint8_t writeByte(uint8_t value);
    void updateRegisterBit(uint8_t pin, uint8_t value, uint8_t portA, uint8_t portB);
    bool bitRead(uint8_t num, uint8_t index);
    void bitWrite(uint8_t &var, uint8_t index, uint8_t bit);
};
#endif
