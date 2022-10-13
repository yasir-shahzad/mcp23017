#include "MCP23017.h"
#include <time.h>
#include <iostream>
#include <unistd.h>

using namespace std;

//constants set to emulate Arduino pin parameters
#define HIGH 1
#define LOW  0

#define INPUT  1
#define OUTPUT 0

#define CHANGE  0
#define FALLING 1
#define RISING  2

// Basic pin reading and pullup test for the MCP23017 I/O expander
// public domain!

// Connect pin #12 of the expander to Analog 5 (i2c clock)
// Connect pin #13 of the expander to Analog 4 (i2c data)
// Connect pins #15, 16 and 17 of the expander to ground (address selection)
// Connect pin #9 of the expander to 5V (power)
// Connect pin #10 of the expander to ground (common ground)
// Connect pin #18 through a ~10kohm resistor to 5V (reset pin, active low)

// Input #0 is on pin 21 so connect a button or switch from there to ground

int main() {
    MCP23017 mcp(2, 0x20); // use default address 0x20
    cout << mcp.openI2C() << "\n";

    for (int i = 0; i < 16; i++) {
        mcp.pinMode(i, OUTPUT);
    }
    while (true)
    {
        for (int i = 0; i < 16; i++) {
            mcp.digitalWrite(i, HIGH);
        }
        usleep(2000000);
        for (int i = 0; i < 16; i++) {
            mcp.digitalWrite(i, LOW);
        }
        usleep(2000000);
    }

    return 0;
}
