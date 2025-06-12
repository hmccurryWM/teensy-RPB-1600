#include <Arduino.h>
#include <HardwareSerial.h>
#include <cmath>
#include <cstdint>
#include "PMBus_ABIO.h"
#include "HEP-1000-commands.h"

#define DEBUG_PRINT

#ifdef DEBUG_PRINT
    #define dprintf(...) Serial.printf(__VA_ARGS__)
#else
    #define dprintf(...)
#endif


#if (ARDUINO >= 157) && !defined(ARDUINO_STM32_FEATHER)
#define SETWIRECLOCK i2c_dev->setSpeed(wireClk)    ///< Set before I2C transfer
#define RESWIRECLOCK i2c_dev->setSpeed(restoreClk) ///< Restore after I2C xfer
#else // setClock() is not present in older Arduino Wire lib (or WICED)
#define SETWIRECLOCK ///< Dummy stand-in define
#define RESWIRECLOCK ///< keeps compiler happy
#endif

#define TRANSACTION_START SETWIRECLOCK
#define TRANSACTION_END RESWIRECLOCK

//#define RPB_1600_DEBUG

#define NL Serial.print(F("\n"));


//----------------------------------------------------------------------
// Public Functions
//----------------------------------------------------------------------

// This entire class assumes I2C hardware is available. Does not yet attempt bitbanging as a 
// fallback
PMBus_ABIO::PMBus_ABIO(TwoWire *wire, uint32_t clkDuring, 
                        uint32_t clkAfter)
#if ARDUINO >= 157
      : wireClk(clkDuring), restoreClk(clkAfter)
#endif
{}

/*!
    @brief  Destructor for PMBus_ABIO object.
*/
PMBus_ABIO::~PMBus_ABIO(void)
{
    if (rxbuffer)
    {
        Serial.println("rxbuffer exists");
        free(rxbuffer->buffer);
        free(rxbuffer);
        rxbuffer = NULL;
    }

    if (data)
    {
        Serial.println("mfr data exists");
        free(data);
        data = NULL;
    }

    if (capa)
    {
        Serial.println("capabilitiy exists");
        free(capa);
        capa = NULL;
    }

    if (internalStatus)
    {
        Serial.println("status exists");
        free(internalStatus);
        internalStatus = NULL;
    }

    if (Operations)
    {
        Serial.println("Operations exists");
        free(Operations);
        Operations = NULL;
    }
}

uint8_t PMBus_ABIO::Init(uint8_t i2caddr)
{
    pmbus_addr = i2caddr;

    //Serial.println("Before buffer length");

    rxbuffer = new buffer_data;
    capa = new capability;
    data = new mfr_data;
    internalStatus = new PMBusStatus();
    Operations = new operationByte();

    rxbuffer->bufferLength = 0;

    //Serial.println("Set address and buffer length init");

    for (int i = 0; i < MAX_RECEIVE_BYTES; i++)
    {
        rxbuffer->buffer[i] = 0;
    }

    delete (i2c_dev);

    Serial.println("deleted i2c");

    i2c_dev = new Adafruit_I2CDevice(pmbus_addr, &Wire);

    if (!i2c_dev->begin()) 
    {
          return NOTBEGUNERR;
    }

#ifdef RPB_1600_DEBUG
    //dprintf("SDA Pin: %d, SCL Pin: %d\n", SDA, SCL);
    Serial.print(F("<RPB-1600 DEBUG> Init complete!\n"));
#endif
    return 0;
}

bool PMBus_ABIO::getReadings(readings *data)
{
    if (!readWithCommand(CMD_CODE_READ_VIN, CMD_LENGTH_READ_VIN))
    {
        return false;
    }

    // vin is not important at all for my current application, so ints are precise enough
    data->v_in = parseLinearData();

    if (!readWithCommand(CMD_CODE_READ_VOUT, CMD_LENGTH_READ_VOUT))
    {
        return false;
    }

    data->v_out = parseLinearVoltage(CMD_N_VALUE_READ_VOUT);

    /*if (!readWithCommand(CMD_CODE_READ_IOUT, CMD_LENGTH_READ_IOUT))
    {
        return false;
    }*/

    float outcurrent = readOutputCurrent();

    dprintf("getreading Output Current is %.4f A\n", outcurrent);

    data->i_out = outcurrent;
/*
    if (!readWithCommand(CMD_CODE_READ_FAN_SPEED_1, CMD_LENGTH_READ_FAN_SPEED_1))
    {
        return false;
    }

    data->fan_speed_1 = parseLinearData();

    if (!readWithCommand(CMD_CODE_READ_FAN_SPEED_2, CMD_LENGTH_READ_FAN_SPEED_2))
    {
        return false;
    }

    data->fan_speed_2 = parseLinearData();
*/
    return true;
}
#ifdef USECHARGER
bool PMBus_ABIO::getChargeStatus(charge_status *status)
{
    if (!readWithCommand(CMD_CODE_CHG_STATUS, CMD_LENGTH_CHG_STATUS))
    {
        return false;
    }

    parseChargeStatus(status);

    return true;
}

bool PMBus_ABIO::getCurveParams(curve_parameters *params)
{
    /* Query the charger for all charge curve related commands, and populate the following items:
    params->cc;
    params->cv;
    params->floating_voltage;
    params->taper_current;
    params->config;
    params->cc_timeout;
    params->cv_timeout;
    params->status; */

    if (!readWithCommand(CMD_CODE_CURVE_CC, CMD_LENGTH_CURVE_CC))
    {
        return false;
    }

    params->cc = parseLinearData();

    if (!readWithCommand(CMD_CODE_CURVE_CV, CMD_LENGTH_CURVE_CV))
    {
        return false;
    }

    params->cv = parseLinearVoltage(CMD_N_VALUE_CURVE_CV);

    if (!readWithCommand(CMD_CODE_CURVE_FV, CMD_LENGTH_CURVE_FV))
    {
        return false;
    }

    params->floating_voltage = parseLinearVoltage(CMD_N_VALUE_CURVE_FV);

    if (!readWithCommand(CMD_CODE_CURVE_TC, CMD_LENGTH_CURVE_TC))
    {
        return false;
    }

    params->taper_current = parseLinearData();

    if (!readWithCommand(CMD_CODE_CURVE_CONFIG, CMD_LENGTH_CURVE_CONFIG))
    {
        return false;
    }

    parseCurveConfig(&params->config);

    if (!readWithCommand(CMD_CODE_CURVE_CC_TIMEOUT, CMD_LENGTH_CURVE_CC_TIMEOUT))
    {
        return false;
    }

    params->cc_timeout = parseLinearData();

    if (!readWithCommand(CMD_CODE_CURVE_CV_TIMEOUT, CMD_LENGTH_CURVE_CV_TIMEOUT))
    {
        return false;
    }

    params->cv_timeout = parseLinearData();

    if (!readWithCommand(CMD_CODE_CURVE_FLOAT_TIMEOUT, CMD_LENGTH_CURVE_FLOAT_TIMEOUT))
    {
        return false;
    }

    params->float_timeout = parseLinearData();

    getChargeStatus(&params->status);

    return true;
}
#endif
uint8_t PMBus_ABIO::readWithCommand(uint8_t commandID, uint8_t receiveLength)
{
    TRANSACTION_START;

#ifdef RPB_1600_DEBUG
    //dprintf("Current Timeout is %d\n ms", Wire.getTimeOut());
    Serial.print(F("<RPB-1600 DEBUG> Attempting to read command "));
    Serial.print(commandID);
    Serial.print(F(" with length "));
    Serial.print(receiveLength);
    Serial.print(F("\n"));
#endif
    
    dprintf("Beginning with command ID 0x%x.\n", commandID);

    //So I might've modified wire? if so, this will need to be defined for brand new libraries
    // write is needed to add the command to the tx buffer, as its protected in wire.
    #define NEEDWRITE
#ifdef NEEDWRITE
    //Serial.println("Setting aio pin high for write");
    //digitalWrite(4, HIGH);

    // handle to the real command ID, since we're reading the length is always 1 byte. false 
    // is to get a repeated start condition so the PSU doesnt freak tf out
    if (!i2c_dev->write(&commandID, 1, false)) // Write to I2C Tx buffer
    {
        return NOWRITEERR;
    }

    //Serial.println("Bringing aio pin back low after write");
    //digitalWrite(4, LOW);

    //Serial.println("Ending transmission of command id");
#endif
    // Clear out buffer in preparation for new data
    clearRXBuffer();

    //Serial.println("Setting aio pin high for read");
    //digitalWrite(4, HIGH);

    // Request bytes from the device
    readFromWrapper(receiveLength);

    //Serial.println("Bringing aio pin back low after read");
    //digitalWrite(4, LOW);
    // Count the number of bytes we receive
    uint8_t num_bytes = rxbuffer->bufferLength;

#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Total bytes received: "));
    Serial.println(num_bytes);
    Serial.print("\n");

    Serial.print(F("<RPB-1600 DEBUG> RX Buffer: ["));
    for (int i = 0; i < num_bytes; i++)
    {
        Serial.print("0x");
        if (rxbuffer->buffer[i] < 0x10)  // Add leading zero if needed
        {
            Serial.print("0");
        }
        Serial.print(rxbuffer->buffer[i], HEX);
        if (i < num_bytes - 1)
        {
            Serial.print(",");
        }
    }
    Serial.print("]\n");

#endif

    TRANSACTION_END;
    dprintf("Number of bytes: %d\n", num_bytes);
    return num_bytes;
}

// NOTE not actually only 2 bytes
uint8_t PMBus_ABIO::writeTwoBytes(uint8_t commandID, uint8_t *data, uint8_t length)
{
#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Attempting to write "));
    Serial.print(data[0]);
    Serial.print(F(" (low) and "));
    Serial.print(data[1]);
    Serial.print(F("(high) with command 0x"));
    Serial.print(commandID);
    NL
#endif

    for (int i = 0; i < length; i++)
    {
        dprintf("data[%d] is 0x%02x\n", i, data[i]);
    }
    dprintf("Attempting to write...\n");

    //Wire.beginTransmission(my_charger_address);
    //Wire.write(commandID);
    //uint8_t bytes_written = Wire.write(data, 2);
    //Wire.endTransmission();

    bool returnstat = i2c_dev->write(data, length, true, &commandID, 1);

    dprintf("returned a val of %d\n", returnstat);

    if (returnstat) 
    {
        return returnstat;
        
    }
    else 
    {
        dprintf("ERROR id");
        return  0;
    }

    return 0xFF;
}

bool PMBus_ABIO::writeVoutTrim(float trim, int8_t N, float lowerBound = TRIM_LOWER_BOUND, float upperBound = TRIM_UPPER_BOUND)
{
    int16_t Y = 0;
    float fY = 0.0;
    int16_t denominator = 0x0001;
    uint8_t txData[2] = { 0 };
    
    if ((lowerBound > trim) || (trim > upperBound))
    {
        dprintf("Error: trim value out of bounds: ");

        if (trim < lowerBound)
            dprintf("%.2f < %.2f\n", trim, lowerBound);
        else if (trim > upperBound)
            dprintf("%.2f > %.2f\n", trim, upperBound);
        else
            dprintf("Misc error? \n");

        return false;
    }

    if (N > 0)
    {
        denominator <<= N;
        Y = (int16_t)(trim / denominator);
    }
    else if (N < 0)
    {
        denominator <<= (-N);
        fY = trim * denominator;
        Y = (int16_t) fY;
        dprintf("float-Y is %.4f and int-Y is %d\n", fY, Y);
    }

    dprintf("Using binary logic, the mantissa is %d\n", Y);

    txData[0] = Y & 0x00FF;
    txData[1] = Y >> 8;

    if (!writeTwoBytes(CMD_CODE_VOUT_TRIM, txData))
    {
        dprintf("Error writing\n");
        return false;
    }

    uint16_t rx = 0;

    readWithCommand(CMD_CODE_VOUT_TRIM, CMD_LENGTH_VOUT_TRIM);

    rx = (rxbuffer->buffer[1] << 8) | (rxbuffer->buffer[0]);

    dprintf("Set vout is now %d, %d was written.\n", rx, Y);

    return true;
}

// This seems to be only if you're interested in integer values... might deprecate?
bool PMBus_ABIO::writeLinearDataCommand(uint8_t commandID, int8_t N, float value)
{
    uint16_t Y = 0;
    uint16_t denominator = 0x0001;
    // The Y value is calculated using the following equation
    // (reference PMBUS spec rev 1.1 section 7.1): Value = Y * 2 ^ N
    // Y = Value / (2 ^ N)
    // We'll calculate denominator and do the math based on the polarity of N

    if (N > 0)
    {
        denominator <<= N;
        Y = (uint16_t) (value / denominator);
    }
    else if (N < 0)
    {
        denominator <<= (-N);
        Y = (uint16_t) (value * denominator);
    }

#define RPB_1600_DEBUG
#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Y calculation: Value = "));
    Serial.print(value);
    Serial.print(F(" Denom = "));
    Serial.print(denominator);
    Serial.print(F(" Y = "));
    Serial.print(Y);
    Serial.print("\n"); 
#endif
#undef RPB_1600_DEBUG

    return writeLinearDataHelper(commandID, N, Y);
}

bool PMBus_ABIO::setOCFaultLimit(float currentLimit, int8_t N)
{
    if (!writeLinearDataCommand(CMD_CODE_IOUT_OC_FAULT_LIMIT, N, currentLimit))
    {
        dprintf("ERROR WRITING IN setOCFaultLimit\n");

        return false;
    }

    dprintf("Success in setOCFaultLimit.\n");

    return true;
}

bool PMBus_ABIO::setControl(uint8_t val, SystemConfig item)
{
    uint8_t hibyte = 0;
    uint8_t lobyte = 0;
    uint8_t EEP_OFF_VAL = 0;
    uint8_t EEP_CONFIG_VAL = 0;
    uint8_t OPERATION_INIT_VAL = 0;
    uint8_t PM_CTRL_VAL = 0;
    uint8_t bytesWritten = 0;

    uint8_t txbuffer[2] = {0x00, 0x00};

    Serial.println("Setting analog/digital control");
    int sizeofthing = CMD_LENGTH_SYSTEM_CONFIG;

    if (!readWithCommand(CMD_CODE_SYSTEM_CONFIG, CMD_LENGTH_SYSTEM_CONFIG))
    {
        Serial.println("ERROR");
        return false;
    }

    hibyte = rxbuffer->buffer[1];
    lobyte = rxbuffer->buffer[0];

    dprintf("Low Byte is 0x%x\n", lobyte);
    dprintf("High Byte is 0x%x\n", hibyte);

    EEP_OFF_VAL = (hibyte >> SYSTEM_CONFIG_EEP_OFF) & SYSTEM_CONFIG_EEP_OFF_MASK;
    EEP_CONFIG_VAL = (hibyte >> SYSTEM_CONFIG_EEP_CONFIG) & SYSTEM_CONFIG_EEP_CONFIG_MASK;
    OPERATION_INIT_VAL = (lobyte >> SYSTEM_CONFIG_OPERATION_INIT) & SYSTEM_CONFIG_OPERATION_INIT_MASK;
    PM_CTRL_VAL = (lobyte >> SYSTEM_CONFIG_PM_CTRL) & SYSTEM_CONFIG_PM_CTRL_MASK;

    if (EEP_OFF_VAL == 1)
        Serial.println("Parameters are not saved into the EEPROM (EEP_OFF == 1)");
    else if (EEP_OFF_VAL == 0)
        Serial.println("Parameters are to be saved into the EEPROM (EEP_OFF == 0)");
    else
        dprintf("ERROR: EEP_OFF is an invalid value: %x\n", EEP_OFF);

    if (EEP_CONFIG_VAL == 0)
        Serial.println("Parameters are saved to EEPROM immediately (EEP_CONFIG = 0)");
    else if (EEP_CONFIG_VAL == 1)
        Serial.println("Parameters are saved to EEPROM when unchanged for 1 minute (EEP_CONFIG = 1)");
    else if (EEP_CONFIG_VAL == 2)
        Serial.println("Parameters are sved to EEPROM when unchanged for 10 minutes (EEP_CONFIG = 2)");
    else
        dprintf("ERROR: EEP_CONFIG is an invalid value: %x\n", EEP_CONFIG);
    
    if (OPERATION_INIT_VAL == 0)
        Serial.println("Power supply is off on startup (OPERATION_INIT = 0)");
    else if (OPERATION_INIT_VAL == 1)
        Serial.println("Power supply is on on startup (OPERATION_INIT = 1)");
    else if (OPERATION_INIT_VAL == 2)
        Serial.println("Power supply starts with whatever it was last time (OPERATION_INIT = 2)");
    else
        dprintf("ERROR: OPERATION_INIT is an invalid value: %x\n", OPERATION_INIT);

    if (PM_CTRL_VAL == 1)
        Serial.println("Power supply is on digital control (PM_CTRL == 1)");
    else if (PM_CTRL_VAL == 0)
        Serial.println("Power supply is on analog control (PM_CTRL == 0)");
    else
        dprintf("ERROR: PM_CTRL is an invalid value: %x\n", PM_CTRL);

    if (item == SC_NONE)
        return true;

    else if (item == OPERATION_INIT)
    {
        if (val != 0 && OPERATION_INIT_VAL == 0)
        {
            Serial.println("Switching from start off to start on");
            txbuffer[1] = hibyte;
            lobyte &= !(0x06);
            txbuffer[0] = lobyte || 0x02;
            dprintf("Lobyte is now %x\n", txbuffer[1]);
            bytesWritten = writeTwoBytes(CMD_CODE_SYSTEM_CONFIG, txbuffer);
            dprintf("Wrote %d bytes.\n", bytesWritten);

            return true;
        }
        else if (val != 1 && OPERATION_INIT_VAL == 1)
        {
            Serial.println("Switching from start on to start off");
            txbuffer[1] = hibyte;
            lobyte &= !(0x06);
            txbuffer[0] = lobyte || 0x00;
            dprintf("Lobyte is now %x\n", txbuffer[1]);
            //stub
            return true;
        }
        else 
        {
            Serial.println("Control is already whatever");
            return true;
        }
    }
    else if (item == PM_CTRL)
    {
        if (val == 1 && PM_CTRL_VAL == 0)
        {
            Serial.println("Switching from analog to digital");
            txbuffer[1] = hibyte;
            lobyte &= !(SYSTEM_CONFIG_PM_CTRL_MASK << SYSTEM_CONFIG_PM_CTRL);
            txbuffer[0] = lobyte || 0x01;
            dprintf("Lobyte is now %x\n", txbuffer[1]);
            bytesWritten = writeTwoBytes(CMD_CODE_SYSTEM_CONFIG, txbuffer);
            dprintf("Wrote %d bytes.\n", bytesWritten);

            return true;
        }
        else if (val == 0 && PM_CTRL_VAL == 1)
        {
            Serial.println("Switching from digital to analog");
            txbuffer[1] = hibyte;
            txbuffer[0] = lobyte || 0x00;
            dprintf("Lobyte is now %x\n", txbuffer[1]);
            bytesWritten = writeTwoBytes(CMD_CODE_SYSTEM_CONFIG, txbuffer);
            dprintf("Wrote %d bytes.\n", bytesWritten);
            return true;
        }
        else 
        {
            Serial.println("Control is already whatever");
            return true;
        }
    }
    else if (item == EEP_OFF) 
    {
        if (val == 1)
        {
            Serial.println("Disabling EEP writes");
            txbuffer[0] = lobyte;
            uint8_t temp = SYSTEM_CONFIG_EEP_OFF_MASK << SYSTEM_CONFIG_EEP_OFF;
            dprintf("0x%02x\n", temp);
            temp = ~(temp);
            dprintf("0x%02x\n", temp);
            temp = ~(SYSTEM_CONFIG_EEP_OFF_MASK << SYSTEM_CONFIG_EEP_OFF);
            dprintf("0x%02x\n", temp);
            temp = hibyte & temp;
            dprintf("0x%02x\n", temp);
            uint8_t tempos = 1 << SYSTEM_CONFIG_EEP_OFF;
            dprintf("0x%02x\n", tempos);
            temp = temp || tempos;
            dprintf("0x%02x\n", temp);
            txbuffer[1] = (hibyte & ~(SYSTEM_CONFIG_EEP_OFF_MASK << SYSTEM_CONFIG_EEP_OFF)) || (1 << SYSTEM_CONFIG_EEP_OFF);
            dprintf("Hibyte is now 0x%02x\n", hibyte);
            //bytesWritten = writeTwoBytes(CMD_CODE_SYSTEM_CONFIG, txbuffer);
            //dprintf("Wrote %d bytes.\n", bytesWritten);
            return true;
        }
        else if (val == 0)
        {
            Serial.println("Enabling EEP writes");
            txbuffer[0] = lobyte;
            txbuffer[1] = (hibyte & !(SYSTEM_CONFIG_EEP_OFF_MASK << SYSTEM_CONFIG_EEP_OFF)) || (0 << SYSTEM_CONFIG_EEP_OFF);
            dprintf("Hibyte is now 0x%02x\n", hibyte);
            //bytesWritten = writeTwoBytes(CMD_CODE_SYSTEM_CONFIG, txbuffer);
            //dprintf("Wrote %d bytes.\n", bytesWritten);
            return true;
        }
    }

    return false;
}

bool PMBus_ABIO::readManufData(mfr_data *mfrdata)
{
    // ID, Model, Revision, Location, Date, Serial
    Serial.println(F("Reading Manu Data"));
    int sizeofthing = CMD_LENGTH_MFR_ID;

    if (!readWithCommand(CMD_CODE_MFR_ID, CMD_LENGTH_MFR_ID))
    {
        Serial.println(F("ERROR"));
        return false;
    }
    #ifdef RPB_1600_DEBUG
        Serial.print(F("Sizeofthing: "));
        Serial.println(sizeofthing);
        Serial.println(F("ID Chars: "));
    #endif

    for (int i = 0; i < sizeofthing; ++i)
    {
#ifdef RPB_1600_DEBUG
        Serial.print(F("Num: "));
        Serial.print(rxbuffer->buffer[i]);
        Serial.print("  Char: ");
            Serial.print((char)rxbuffer->buffer[i]);
            Serial.print(F("\n"));
#endif
        mfrdata->id[i] = rxbuffer->buffer[i];
    }

    sizeofthing = CMD_LENGTH_MFR_MODEL;

    if (!readWithCommand(CMD_CODE_MFR_MODEL, CMD_LENGTH_MFR_MODEL))
    {
        return false;
    }
#ifdef RPB_1600_DEBUG
    Serial.print(F("Sizeofthing: "));
    Serial.println(sizeofthing);
    Serial.println(F("ID Chars: "));
#endif

    for (int i = 0; i < sizeofthing; ++i)
    {
#ifdef RPB_1600_DEBUG
         Serial.print(F("Num: "));
         Serial.print(rxbuffer->buffer[i]);
         Serial.print("  Char: ");
             Serial.print((char)rxbuffer->buffer[i]);
             Serial.print(F("\n"));
#endif
        mfrdata->model[i] = rxbuffer->buffer[i];
    }

    sizeofthing = CMD_LENGTH_MFR_REVISION;

    if (!readWithCommand(CMD_CODE_MFR_REVISION, CMD_LENGTH_MFR_REVISION))
    {
        return false;
    }
#ifdef RPB_1600_DEBUG
    Serial.print(F("Sizeofthing: "));
    Serial.println(sizeofthing);
    Serial.println(F("ID Chars: "));
#endif

    for (int i = 0; i < sizeofthing; ++i)
    {
#ifdef RPB_1600_DEBUG
         Serial.print(F("Num: "));
         Serial.print(rxbuffer->buffer[i]);
         Serial.print("  Char: ");
             Serial.print((char)rxbuffer->buffer[i]);
             Serial.print(F("\n"));
#endif
        mfrdata->revision[i] = rxbuffer->buffer[i];
    }

    sizeofthing = CMD_LENGTH_MFR_LOCATION;

    if (!readWithCommand(CMD_CODE_MFR_LOCATION, CMD_LENGTH_MFR_LOCATION))
    {
        return false;
    }
#ifdef RPB_1600_DEBUG
    Serial.print(F("Sizeofthing: "));
    Serial.println(sizeofthing);
    Serial.println(F("ID Chars: "));
#endif

    for (int i = 0; i < sizeofthing; ++i)
    {
#ifdef RPB_1600_DEBUG
         Serial.print(F("Num: "));
         Serial.print(rxbuffer->buffer[i]);
         Serial.print("  Char: ");
             Serial.print((char)rxbuffer->buffer[i]);
             Serial.print(F("\n"));
#endif
        mfrdata->location[i] = rxbuffer->buffer[i];
    }

    sizeofthing = CMD_LENGTH_MFR_DATE;

    if (!readWithCommand(CMD_CODE_MFR_DATE, CMD_LENGTH_MFR_DATE))
    {
        return false;
    }
#ifdef RPB_1600_DEBUG
    Serial.print(F("Sizeofthing: "));
    Serial.println(sizeofthing);
    Serial.println(F("ID Chars: "));
#endif

    for (int i = 0; i < sizeofthing; ++i)
    {
#ifdef RPB_1600_DEBUG
         Serial.print(F("Num: "));
         Serial.print(rxbuffer->buffer[i]);
         Serial.print("  Char: ");
             Serial.print((char)rxbuffer->buffer[i]);
             Serial.print(F("\n"));
#endif
        mfrdata->date[i] = rxbuffer->buffer[i];
    }
    
    sizeofthing = CMD_LENGTH_MFR_SERIAL;

    if (!readWithCommand(CMD_CODE_MFR_SERIAL, CMD_LENGTH_MFR_SERIAL))
    {
        return false;
    }
#ifdef RPB_1600_DEBUG
    Serial.print(F("Sizeofthing: "));
    Serial.println(sizeofthing);
    Serial.println(F("ID Chars: "));
#endif

    for (int i = 0; i < sizeofthing; i++)
    {
#ifdef RPB_1600_DEBUG
        Serial.print(F("Num: "));
        Serial.print(rxbuffer->buffer[i]);
        Serial.print("  Char: ");
        Serial.print((char)rxbuffer->buffer[i]);
        Serial.println(" ");
#endif
        mfrdata->serial[i] = rxbuffer->buffer[i];
    }

    return true;
}

//----------------------------------------------------------------------
// Private Functions
//----------------------------------------------------------------------

bool PMBus_ABIO::writeLinearDataHelper(uint8_t commandID, int8_t N, int16_t Y)
{
    // Make sure the N value isn't bigger then 5 bits
    if (abs(N) > 15)
    {
#ifdef RPB_1600_DEBUG
        Serial.print(F("<RPB-1600 DEBUG> N value too large! Can't convert to linear format. N = "));
        Serial.println(N);
        NL
#endif
        return false;
    }

    // Make sure the Y (Mantissa) can fit in 11 bits
    if (abs(Y) > 2047)
    {
#ifdef RPB_1600_DEBUG
        Serial.print(F("<RPB-1600 DEBUG> Mantissa (Y) value too large! Can't convert to linear format. Y = "));
        Serial.println(Y);
#endif
        return false;
    }

    // The goal here is to truncate Y to 11 bits, and put those in the lowest 11 bits of the outgoing data
    // Then we need to truncate N to 5 bits, and put those bits in the highest 5 bits of the outgoing data
    // See Section 7.1 of the PMBus 1.1 specification for more info on the "Linear Data Format"
    // Note that data[0] is the low byte (sent first) and data[1] is the high byte (sent second)
    uint8_t data[2] = {0x00, 0x00};

    // Mask the lowest 8 bits of the mantissa, and put those bits in the low byte of the outgoing data
    data[0] = Y & 0x00FF;
    // Mask the lowest 3 bits of the high byte of the mantissa, and put those bits in the high byte of the outgoing data
    data[1] = (Y & 0x0700) >> 8;
    // Mask the lowest 5 bits of N, and shift them to be the highest 5 bytes of the high byte
    data[1] = (N & 0x1F) << 3;
#ifdef RPB_1600_DEBUG
    Serial.print("<RPB-1600 DEBUG> Attempting to write linear data with N = ");
    Serial.print(N);
    Serial.print(" and Mantissa (Y) = ");
    Serial.println(Y);
#endif
    writeTwoBytes(commandID, data);

    return true;
}

float PMBus_ABIO::readOutputCurrent()
{
    if (!readWithCommand(CMD_CODE_READ_IOUT, CMD_LENGTH_READ_IOUT))
    {
        Serial.println("Error reading current");
        return -1.0;
    }

    float readcurrent = parseOutputCurrent();

    dprintf("Read output current is %.4f A\n", readcurrent);

    return readcurrent;
}


float PMBus_ABIO::parseOutputCurrent()
{
    uint16_t rawData = rxbuffer->buffer[0] | (rxbuffer->buffer[1] << 8);

    // Mask & shift out the "N" and "mantissa" values and convert them from 2s complement
    int16_t rawN = (rawData & N_EXPONENT_MASK) >> N_EXPONENT_SHIFT;
    int16_t N = UpscaleTwosComplement(rawN, N_EXPONENT_LENGTH);
    int16_t rawMantissa = (rawData & MANTISSA_MASK);
    
#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Raw Data: 0x"));
    Serial.println(rawData, HEX);

    Serial.print(F("<RPB-1600 DEBUG> Raw N: 0x"));
    Serial.println(rawN, HEX);

    Serial.print(F("<RPB-1600 DEBUG> N: "));
    Serial.println(N);

    Serial.print(F("<RPB-1600 DEBUG> Raw Mantissa : 0x"));
    Serial.println(rawMantissa, HEX);
#endif

    float result = rawMantissa * std::pow(2.0, N);

    dprintf("Parsed Output current: %.4f A\n", result);

    return result;
}

uint16_t PMBus_ABIO::parseLinearData(void)
{
    uint16_t rawData = rxbuffer->buffer[0] | (rxbuffer->buffer[1] << 8);

    // Mask & shift out the "N" and "mantissa" values and convert them from 2s complement
    int16_t rawN = (rawData & N_EXPONENT_MASK) >> N_EXPONENT_SHIFT;
    int16_t N = UpscaleTwosComplement(rawN, N_EXPONENT_LENGTH);
    int16_t rawMantissa = (rawData & MANTISSA_MASK);
    int16_t mantissa = UpscaleTwosComplement(rawMantissa, MANTISSA_LENGTH);

    uint16_t result = static_cast<uint16_t>(mantissa);
    // The result is calculated by doing the following: result = mantissa * 2 ^ N
    // however, N can be positive or negative, so we shift the mantissa accordingly
    // to perform the multiplication by a power of 2.
    if (N > 0)
    {
        result <<= N;
    }
    else if (N < 0)
    {
        result >>= (-N);
    }
#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Raw Data: 0x"));
    Serial.println(rawData, HEX);

    Serial.print(F("<RPB-1600 DEBUG> Raw N: 0x"));
    Serial.println(rawN, HEX);

    Serial.print(F("<RPB-1600 DEBUG> N: "));
    Serial.println(N);

    Serial.print(F("<RPB-1600 DEBUG> Raw Mantissa : 0x"));
    Serial.println(rawMantissa, HEX);

    Serial.print(F("<RPB-1600 DEBUG> Mantissa: "));
    Serial.println(mantissa);

    Serial.print(F("<RPB-1600 DEBUG> Result: "));
    Serial.println(result);

#endif

    return result;
}

float PMBus_ABIO::parseLinearVoltage(int8_t N)
{
    uint16_t rawData = rxbuffer->buffer[0] | (rxbuffer->buffer[1] << 8);

    // Calculate divisor using N
    uint16_t divisor = 0x01 << abs(N);

#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Parsing linear voltage w/N: "));
    Serial.println(N);
    NL
#endif

    float result = (float)rawData;

    // Divide based on the polarity of N
    if (N > 0)
    {
        result /= -divisor;
    }
    else if (N < 0)
    {
        result /= divisor;
    }

#ifdef RPB_1600_DEBUG
    Serial.print(F("<RPB-1600 DEBUG> Linear voltage result: "));
    Serial.println(result);
    NL
#endif

    return result;
}
#ifdef USECHARGER
void PMBus_ABIO::parseCurveConfig(curve_config *config)
{
    // Bits 0 & 1 of low byte
    config->charge_curve_type = (my_rx_buffer[0] & 0x02);
    // Bits 2 & 3 of low byte
    config->temp_compensation = (my_rx_buffer[0] & 0x0C) >> 2;
    // Bit 7 of low byte (0 = 3 stage, 1 = 2 stage)
    config->num_charge_stages = (my_rx_buffer[0] && 0x40) ? 2 : 3;
    // Bit 0 of high byte
    config->cc_timeout_indication_enabled = (my_rx_buffer[1] && 0x01);
    // Bit 1 of high byte
    config->cv_timeout_indication_enabled = (my_rx_buffer[1] && 0x02);
    // Bit 2 of high byte
    config->float_stage_timeout_indication_enabled = (my_rx_buffer[1] && 0x04);
} 

void PMBus_ABIO::parseChargeStatus(charge_status *status)
{
    // Low byte:
    status->fully_charged = (my_rx_buffer[0] && 0x01); // Bit 0
    status->in_cc_mode = (my_rx_buffer[0] && 0x02);    // Bit 1
    status->in_cv_mode = (my_rx_buffer[0] && 0x04);    // Bit 2
    status->in_float_mode = (my_rx_buffer[0] && 0x08); // Bit 3
    // High byte:
    status->EEPROM_error = (my_rx_buffer[1] && 0x01);                    // Bit 0
    status->temp_compensation_short_circuit = (my_rx_buffer[1] && 0x04); // Bit 1
    status->battery_detected = (my_rx_buffer[1] && 0x08);                // Bit 3
    status->timeout_flag_cc_mode = (my_rx_buffer[1] && 0x20);            // Bit 5
    status->timeout_flag_cv_mode = (my_rx_buffer[1] && 0x40);            // Bit 6
    status->timeout_flag_float_mode = (my_rx_buffer[1] && 0x80);         // Bit 7
}
#endif
// Slightly modified version of this https://www.codeproject.com/Tips/1079637/Twos-Complement-for-Unusual-Integer-Sizes
int16_t PMBus_ABIO::UpscaleTwosComplement(int16_t value, size_t length)
{
    uint16_t mask = (~0) << length;
    // Too small for complement?
    if (length < 2)
    {
        return (~mask & value);
    }
    // Two's complement?
    uint16_t msb = 1 << (length - 1);
    if (value & msb)
    {
        return (mask | value);
    }
    else
    {
        return (~mask & value);
    }
}

void PMBus_ABIO::clearRXBuffer(void)
{
    
    dprintf("Current buffer length is %d\n", rxbuffer->bufferLength);

    for (int i = 0; i < MAX_RECEIVE_BYTES; i++)
    {
        //dprintf("Buffer index %d is %d\n", i, rxbuffer->buffer[i]);
        rxbuffer->buffer[i] = 0;
    }
    
    rxbuffer->bufferLength = 0;
    Serial.println("Reset buffer length");
#ifdef RPB_1600_DEBUG
    Serial.print("<RPB-1600 DEBUG> RX Buffer Cleared!\n");
#endif
}

// pulls the number of bytes read
bool PMBus_ABIO::readFromWrapper(uint8_t len)
{
    uint8_t* ptr = rxbuffer->buffer;
    dprintf("Reading in %d bytes\n", len);

    if (!i2c_dev->read(ptr, len))
    {
        Serial.println("ERROR READING, FLUSHING BUFFER");
        clearRXBuffer();
        return false;
    }

    Serial.println("Read in data");

    rxbuffer->bufferLength = len;

    return true;
}

bool PMBus_ABIO::returnBufferData(buffer_data* bufferHandle)
{
    uint8_t* ptr = rxbuffer->buffer;

    if (bufferHandle->buffer == nullptr)
    {
        return false;
    }

    for (int i = 0; i < MAX_RECEIVE_BYTES; i++)
    {
        //dprintf("External buffer index %d is %d and internal is %d\n", i, bufferHandle->buffer[i], ptr[i]);
    }
    memcpy(bufferHandle->buffer, ptr, rxbuffer->bufferLength);

    bufferHandle->bufferLength = rxbuffer->bufferLength;
    
    return true;
}

uint8_t PMBus_ABIO::getCapability()
{
    uint8_t capaBuff = 0;

    if (!readWithCommand(CMD_CODE_CAPABILITY, CMD_LENGTH_CAPABILITY))
    {
        return NOREADERR;
    }

    capaBuff = rxbuffer->buffer[0];
    

    dprintf("PMBus 1.1 Capability Byte: 0x%02X\n", capaBuff);

    // Bit 7: Packet Error Checking
    capa->CRC = capaBuff & (1 << 7);
    if (capa->CRC)
        dprintf(" - Packet Error Checking (PEC) supported\n");
    else
        dprintf(" - Packet Error Checking (PEC) not supported\n");

    // Bits 6:5: Bus Speed
    uint8_t bus_speed = (capaBuff >> 5) & 0x03;
    capa->BUS = bus_speed;
    switch (bus_speed) {
        case 0x00:
            dprintf(" - Max Bus Speed: 100 kHz\n");
            break;
        case 0x01:
            dprintf(" - Max Bus Speed: 400 kHz\n");
            break;
        default:
            dprintf(" - Bus speed setting RESERVED (invalid configuration)\n");
            break;
    }

    // Bit 4: SMBALERT#
    capa->ALT = capaBuff & (1 <<4);
    if (capa->ALT)
        dprintf(" - SMBALERT# supported\n");
    else
        dprintf(" - SMBALERT# not supported\n");

    // Bits 3:0 are reserved
    
    return GOOD;
}

uint8_t PMBus_ABIO::returnCapability(capability* data)
{
    data->ALT = capa->ALT;
    data->BUS = capa->BUS;
    data->CRC = capa->CRC;
    return 0;
}

bool PMBus_ABIO::parse_status_word(buffer_data *status_bytes, PMBusStatus *status) 
{
    Serial.println("entering");

    uint16_t word = rxbuffer->buffer[0] | (rxbuffer->buffer[1] << 8);

    dprintf("The Status word is 0x%04x\n", word);

    // STATUS_BYTE bits
    internalStatus->busy               = (word >> STATUS_WORD_BIT_7_BUSY) & STATUS_WORD_BIT_MASK;
    internalStatus->off                = (word >> STATUS_WORD_BIT_6_OFF) & STATUS_WORD_BIT_MASK;
    internalStatus->vout_ov            = (word >> STATUS_WORD_BIT_5_VOUT_OV) & STATUS_WORD_BIT_MASK;
    internalStatus->iout_oc            = (word >> STATUS_WORD_BIT_4_IOUT_OC) & STATUS_WORD_BIT_MASK;
    internalStatus->vin_uv             = (word >> STATUS_WORD_BIT_3_VIN_UV) & STATUS_WORD_BIT_MASK;
    internalStatus->temperature        = (word >> STATUS_WORD_BIT_2_TEMPERATURE) & STATUS_WORD_BIT_MASK;
    internalStatus->cml                = (word >> STATUS_WORD_BIT_1_CML) & STATUS_WORD_BIT_MASK;
    internalStatus->none_of_the_above = (word >> STATUS_WORD_BIT_0_NONE_OF_THE_ABOVE) & STATUS_WORD_BIT_MASK;

    // Upper byte of STATUS_WORD
    internalStatus->vout               = (word >> STATUS_WORD_BIT_15_VOUT) & STATUS_WORD_BIT_MASK;
    internalStatus->iout               = (word >> STATUS_WORD_BIT_14_IOUT) & STATUS_WORD_BIT_MASK;
    internalStatus->input              = (word >> STATUS_WORD_BIT_13_INPUT) & STATUS_WORD_BIT_MASK;
    internalStatus->mfr_specific       = (word >> STATUS_WORD_BIT_12_MFR_SPECIFIC) & STATUS_WORD_BIT_MASK;
    internalStatus->power_good_negated = (word >> STATUS_WORD_BIT_11_POWER_GOOD_N) & STATUS_WORD_BIT_MASK;
    internalStatus->fans               = (word >> STATUS_WORD_BIT_10_FANS) & STATUS_WORD_BIT_MASK;
    internalStatus->other              = (word >> STATUS_WORD_BIT_9_OTHER) & STATUS_WORD_BIT_MASK;
    internalStatus->unknown            = (word >> STATUS_WORD_BIT_8_UNKNOWN) & STATUS_WORD_BIT_MASK;

    return true;
}

bool PMBus_ABIO::pullStatus()
{
    Serial.println("Getting status bytes...");

    if (!readWithCommand(CMD_CODE_STATUS_WORD, CMD_LENGTH_STATUS_WORD))
    {
        return false;
    }

    Serial.println("Read in status command");

    if (!parse_status_word(rxbuffer, internalStatus))
        return false;

    if (!print_status_bits(internalStatus))
        return false;

    return true;
}

bool PMBus_ABIO::print_status_bits(PMBusStatus *status) 
{
    if (!status) 
        return false;

    dprintf("STATUS_BYTE bits (no faults if empty):\n");
    if (status->busy)               dprintf("  BUSY: Device is busy.\n");
    if (status->off)                dprintf("  OFF: Unit is turned off.\n");
    if (status->vout_ov)            dprintf("  VOUT_OV: Output overvoltage fault.\n");
    if (status->iout_oc)            dprintf("  IOUT_OC: Output overcurrent fault.\n");
    if (status->vin_uv)             dprintf("  VIN_UV: Input undervoltage fault.\n");
    if (status->temperature)        dprintf("  TEMPERATURE: Overtemperature fault or warning.\n");
    if (status->cml)                dprintf("  CML: Communications, logic, or memory fault.\n");
    if (status->none_of_the_above)  dprintf("  NONE_OF_THE_ABOVE: Unspecified fault or warning.\n");

    dprintf("STATUS_WORD upper bits (No faults if empty):\n");
    if (status->vout)               dprintf("  VOUT: Output voltage fault/warning.\n");
    if (status->iout)               dprintf("  IOUT: Output current fault/warning.\n");
    if (status->input)              dprintf("  INPUT: Input voltage fault/warning.\n");
    if (status->mfr_specific)       dprintf("  MFR_SPECIFIC: Manufacturer-specific fault/warning.\n");
    if (status->power_good_negated) dprintf("  POWER_GOOD#: Power is NOT good.\n");
    if (status->fans)               dprintf("  FANS: Fan fault or warning.\n");
    if (status->other)              dprintf("  OTHER: Other fault or warning.\n");
    if (status->unknown)            dprintf("  UNKNOWN: Unknown fault or warning.\n");

    dprintf("\n");

    return true;
}

bool PMBus_ABIO::runOperation(OperationFields opfield, uint8_t bitPosition, uint8_t bitMask, uint8_t value)
{
    uint8_t tempOpBuffer = 0;
    uint8_t txBuilder[1] = {0};

    if (!readWithCommand(CMD_CODE_OPERATION, CMD_LENGTH_OPERATION))
    {
        return false;
    }

    tempOpBuffer = (uint8_t) rxbuffer->buffer[rxbuffer->bufferLength - 1];
    dprintf("Operation Register: 0x%02x\n", tempOpBuffer);

    Operations->faultAction = (tempOpBuffer >> OPERATION_FAULT_ACTION) & OPERATION_FAULT_ACTION_MASK;
    Operations->marginCall = (tempOpBuffer >> OPERATION_MARGIN) & OPERATION_MARGIN_MASK;
    Operations->offType = (tempOpBuffer >> OPERATION_OFF_TYPE) & OPERATION_OFF_TYPE_MASK;
    Operations->onOff = (tempOpBuffer >> OPERATION_ON_OFF) & OPERATION_ON_OFF_MASK;

    dprintf("On_Off is %d\n", Operations->onOff);
    if (Operations->onOff == 1)             Serial.println(" PSU is ON");
    else if (!(Operations->onOff))          Serial.println(" PSU is OFF");
    else                                    Serial.println(" On/off has an invalid value");
    
    dprintf("Off_Type is %d\n", Operations->offType);
    if (Operations->offType == 1)           Serial.println(" PSU immediately turns off");
    else if (!(Operations->offType))        Serial.println(" PSU soft turns off");
    else                                    Serial.println(" Off type has an invalid value");

    dprintf("Margin_Call is %d\n", Operations->marginCall);
    if (Operations->marginCall == 0)        Serial.println(" No margin checking");
    else if (Operations->marginCall == 1)   Serial.println(" Checks lower margin/under- conditions");
    else if (Operations->marginCall == 2)   Serial.println(" Checks upper margin/over- conditions");
    else                                    Serial.println(" Margin has an invalid value");

    dprintf("Fault_Action is %d\n", Operations->faultAction);
    if (Operations->faultAction == 1)       Serial.println(" Faults are IGNORED if margin called");
    else if (Operations->faultAction == 2)  Serial.println(" Faults are ACTED UPON if margin called");
    else                                    Serial.println(" Fault action has an invalid value");

    NL

    switch (opfield) 
    {
        case OF_NONE:
            Serial.println("No operation change request.");
            return false;
        case ON_OFF:
            Serial.println("Turning on or off");

            if (value != 1 || value != 0) 
            {
                dprintf("Switching to 0x%02x\n", value);
                uint8_t maskoff = (OPERATION_ON_OFF_MASK << OPERATION_ON_OFF);
                //dprintf("Mask is 0x%02x\n", maskoff);
                maskoff = !(maskoff);
                //dprintf("notted Mask is 0x%02x\n", maskoff);
                txBuilder[0] = (tempOpBuffer & maskoff);
                //dprintf("Operation buffer is 0x%02x\n", txBuilder[0]);
                txBuilder[0] = (value << OPERATION_ON_OFF);
                //dprintf("Operation buffer is 0x%02x\n", txBuilder[0]);
                if(!writeTwoBytes(CMD_CODE_OPERATION, txBuilder , 1))
                {
                    Serial.println("Error writing value");
                    return false;
                }
            }
            else 
            {
                dprintf("Value is invalid at 0x%x", value);
                return false;
            }
            
            // Re-pulls operation byte
            runOperation(OF_NONE, 0, 0, 0);
            return true;
        case OFF_TYPE:
            Serial.println("Off type stubbed");
            return true;
        case MARGIN_CALL:
            Serial.println("Margin stubbed");
            return true;
        case FAULT_ACTION:
            Serial.println("Fault type stubbed");
            return true;
        default:
            Serial.println("Error switch case");
            return false;
    }
    
    return false;
}

/*// Eventually move this into its own hep-1000-100 derived class
bool changeOCFaultLimit(float current)
{
  Serial.printf("Setting the current limit to %.4f A\n", current);

  if (!HAPSU.setOCFaultLimit(current, CMD_N_VALUE_IOUT_OC_FAULT_LIMIT))
  {
    Serial.println("ERROR WRITING LIMIT");
    return false;
  }

  Serial.println("Success")

  return true;
}

// Eventually move this into its own hep-1000-100 derived class
bool changeVout(float vout)
{
  float trim = vout - 100.0;

  Serial.printf("Setting vout to %.2f by writing the trim as %.2f\n", vout, trim);

  if (!HAPSU.writeVoutTrim(trim, CMD_N_VALUE_VOUT_TRIM))
  {
    Serial.println("ERROR Writing vout trim");

    return false;
  }

  Serial.println("Success");

  return true;
}*/