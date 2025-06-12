/* #include <cstdint>
#include <filesystem>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h> */
#include "PMBus_ABIO.h"
#include "HEP-1000-commands.h"

#ifndef HEP_1000_X
#define HEP_1000_X

class HEP_1000 : public PMBus_ABIO
{
protected:
    float vtrim;
    float vout_set;

public:
    bool setVout(float* vout);
    bool getSetVout(float* vout);
private:
    //nothing
};

#endif