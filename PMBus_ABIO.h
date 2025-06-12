#include <cstdint>
#include <filesystem>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

#ifndef PMBus_ABIO_H
#define PMBus_ABIO_H

// Uncomment the below #define to enable debugging print statements.
// NOTE: You must call Serial.being(<baud rate>) in your setup() for this to work
// #define RPB_1600_DEBUG

/**
 * @brief The maximum number of bytes we could possibly expect to receive from the charger
 */
#define MAX_RECEIVE_BYTES 12

#define N_EXPONENT_MASK 0xF800 // Bitmask for pulling out the first 5 bytes of the payload (the N exponent value)
#define MANTISSA_MASK 0x07FF   // Bitmask for pulling out the last 11 bytes of the payload (the Mantissa)
#define N_EXPONENT_LENGTH 5
#define MANTISSA_LENGTH 11
#define N_EXPONENT_SHIFT MANTISSA_LENGTH

#define GOOD 0x01
#define NOTBEGUNERR 0x02
#define NOWRITEERR 0x04
#define NOREADERR 0x08
#define BADBUFFER 0x10

// PMBus v1.1 Capability Bits
#define CRCCAPABILITY 0x80
#define BUSCAPABILITY 0x60
#define ALERTCAPABILITY 0x10

// PMBus v1.1 Operation bits
#define OPERATIONIMMOFF 0X00
#define OPERATIONSOFTOFF 0X40
#define OPERATIONON 0X80
#define OPERATIONONOFFMASK 0X
#define OPERATIONMARGINMASK 0X

struct readings
{
    uint16_t v_in;
    float v_out;
    float i_out;
    //uint16_t fan_speed_1;
    //uint16_t fan_speed_2;
};

struct mfr_data
{
    char id[12];
    char model[12];
    char revision[24];
    char location[3];
    char date[6];
    char serial[12];
};

struct buffer_data
{
    uint8_t buffer[MAX_RECEIVE_BYTES];
    uint8_t bufferLength;
};

struct capability
{
    bool CRC;
    uint8_t BUS;
    bool ALT;
};

struct status
{
    uint8_t onOff;
    uint8_t onOffControl;
};

struct PMBusStatus
{
    // STATUS_BYTE bits (lower 8 bits of STATUS_WORD)
    uint8_t busy;
    uint8_t off;
    uint8_t vout_ov;
    uint8_t iout_oc;
    uint8_t vin_uv;
    uint8_t temperature;
    uint8_t cml;
    uint8_t none_of_the_above;

    // Upper 8 bits of STATUS_WORD
    uint8_t vout;
    uint8_t iout;
    uint8_t input;
    uint8_t mfr_specific;
    uint8_t power_good_negated;
    uint8_t fans;
    uint8_t other;
    uint8_t unknown;
};

struct operationByte
{
    uint8_t onOff;
    uint8_t offType;
    uint8_t marginCall;
    uint8_t faultAction;
};

enum SystemConfig
{
    SC_NONE,
    PM_CTRL,
    OPERATION_INIT,
    EEP_CONFIG,
    EEP_OFF
};

enum OperationFields
{
    OF_NONE,
    ON_OFF,
    OFF_TYPE,
    MARGIN_CALL,
    FAULT_ACTION
};

enum ON_OFF
{
    ON,
    OFF
};

#ifdef USECHARGER
struct curve_config
{
    uint8_t charge_curve_type;
    uint8_t temp_compensation;
    uint8_t num_charge_stages;
    bool cc_timeout_indication_enabled;
    bool cv_timeout_indication_enabled;
    bool float_stage_timeout_indication_enabled;
};

struct charge_status
{
    bool fully_charged;
    bool in_cc_mode;
    bool in_cv_mode;
    bool in_float_mode;
    // From datasheet:
    // When EEPROM Charge Parameter Error occurs, the charger stops
    // charging the battery and the LED indicator turns red. The
    // charger needs to re-power on to re-start charging the battery.
    bool EEPROM_error;
    // From datasheet: When Temperature Compensation Short occurs, the
    // charger output will shut down and the LED indicator will turn red.
    // The charger will automatically restart after the Temperature
    // Compensation Short condition is removed.
    bool temp_compensation_short_circuit;
    // From datasheet: When there is no battery detected, the charger
    // stops charging the battery and the LED indicator turns red. The
    // charger needs to re-power on to re-start charging the battery
    bool battery_detected;
    // From datasheet: When timeout arises in the Constant Current stage,
    // the charger stops charging the battery and the LED indicator turns
    // red. The charger needs to re-power on to re-start charging the
    // battery
    bool timeout_flag_cc_mode;
    // From datasheet: When timeout arises in the Constant Voltage stage,
    // the charger stops charging the battery and the LED indicator turns
    // red. The charger needs to re-power on to re-start charging the
    // battery
    bool timeout_flag_cv_mode;
    // From datasheet: When timeout arises in the Float stage, the
    // charger stops charging the battery and the LED indicator turns
    // green. This charging flow is finished; the charger needs to
    // re-power on to start charging a different battery
    bool timeout_flag_float_mode;
};

struct curve_parameters
{
    uint16_t cc;
    float cv;
    float floating_voltage;
    uint16_t taper_current;
    curve_config config;
    uint16_t cc_timeout;
    uint16_t cv_timeout;
    uint16_t float_timeout;
    charge_status status;
};
#endif
class PMBus_ABIO
{
public:
    // Constructor, twi is the 2-wire interface, the clk during and after is to keep compatibility
    // with other devices that might use different frequencies
    PMBus_ABIO(TwoWire *wire = &Wire, uint32_t clkDuring = 100000UL, uint32_t clkAfter = 100000UL);

    // Destructor
    ~PMBus_ABIO(void);

    /**
     * @brief Buffer to hold bytes received over i2c
     */
    uint8_t my_rx_buffer[MAX_RECEIVE_BYTES];

    uint8_t Init(uint8_t i2caddr);

    /**
     * @brief Query charger for voltage & current readings, populate a "readings" struct
     * @return true on successful read, false otherwise
     */
    bool getReadings(readings *data);
#ifdef USECHARGER
    /**
     * @brief Query charger for status bytes, populate a "charge_status" struct
     * @return true on successful read, false otherwise
     */
    bool getChargeStatus(charge_status *status);

    /**
     * @brief Query charger for curve parameter bytes, populate a "curve_parameters" struct
     * @return true on successful read, false otherwise
     */
    bool getCurveParams(curve_parameters *params);
#endif
    /**
     * @brief Write arbitrary bytes with commandID
     * @return true on successful write, false otherwise
     */
    uint8_t writeTwoBytes(uint8_t commandID, uint8_t *data, uint8_t length = 2);

    // val is desired value (ignored if nothing being written)
    // Item selects the thing being written
    bool setControl(uint8_t val, SystemConfig item);

    /**
     * @brief Write linear value with specified commandID & N
     * @details See the PMBus 1.1 Spec for more info on how the linear data format works
     * @param N the exponent
     * @param value the value you want to write (NOT the Y value)
     * @return True on success, false on failure
     */
    bool writeLinearDataCommand(uint8_t commandID, int8_t N, float value);

    /**
     * @brief Sends commandID to the charger, and reads the receiveLength byte(s) long response into my_rx_buffer[]
     * @return true if we received the number of bytes we were expecting, false otherwise.
     */
    uint8_t readWithCommand(uint8_t commandID, uint8_t receiveLength);

    /**
    * @brief Pulls the mfgr information
    * @return true if we received the number of bytes we were expecting, false otherwise
    */
    bool readManufData(mfr_data *mfrdata);

    /**
     * @brief Parses the first two bytes of my_rx_buffer[] in the "Linear Data" format outlined int the PMBus Specification
     * @details see the PMBus V1.1 Section 7.1 "Linear Data Format" for more info
     */
     uint16_t parseLinearData(void);

     /**
      * @brief Returns the RX buffer
      * @return full RX buffer of type buffer_data
      */
    bool returnBufferData(buffer_data *bufferhandle);

    /**
     * @brief Pulls the capabilities from the device to an internal structure
     * @return returns an error code
     */
    uint8_t getCapability();

    /**
     * @brief Returns the capabilities from the device for nebulous future uses
     * @return returns the capability data object
     */
    uint8_t returnCapability(capability* data);

    bool pullStatus();

    bool print_status_bits(PMBusStatus *status);

    bool runOperation(OperationFields opfield, uint8_t bitPosition, uint8_t bitMask, uint8_t value);

    float parseOutputCurrent();

    float readOutputCurrent();

    bool writeVoutTrim(float trim, int8_t N, float lowerBound, float upperBound);

    bool setOCFaultLimit(float currentLimit, int8_t N);

private:
    Adafruit_I2CDevice *i2c_dev;
    buffer_data *rxbuffer;
    uint8_t runningClk;
    mfr_data *data;
    capability *capa;
    status *stats;
    PMBusStatus *internalStatus;
    operationByte *Operations;

    #if ARDUINO >= 157
  uint32_t wireClk;    ///< Wire speed for SSD1306 transfers
  uint32_t restoreClk; ///< Wire speed following SSD1306 transfers
#endif

    /**
     * @brief The address of the charger we're communicating with
     * @details This is set using the A0, A1, and A2 pins on the RPB-1600. These three pins control
     * the lowest 3 bits of the 7 bit address, and the MSB is always 1. For example, if all the
     * pins are tied high, the address would be 0x47.
     * @note Address 0 is a reserved address.
     */
    uint8_t pmbus_addr;

    /**
     * @brief Helper for writing linear data with a specified commandID
     * @details See the PMBus 1.1 Spec for more info on how the linear data format works
     * @param N the exponent
     * @param Y the mantissa
     * @return True on success, false on failure
     */
    bool writeLinearDataHelper(uint8_t commandID, int8_t N, int16_t Y);

    /**
     * @brief Parse a voltage reading in the linear format
     * @details See the PMBus 1.1 spec section 8.3.1 for more info
     */
    float parseLinearVoltage(int8_t N);
#ifdef USECHARGER
    /**
     * @brief Parses the first two bytes of my_rx_buffer[] into a curve_config struct and returns it via argument.
     * @details Meant to be called after calling readWithCommand(CMD_CODE_CURVE_CONFIG, CMD_LENGTH_CURVE_CONFIG)
     */
    void parseCurveConfig(curve_config *config);

    /**
     * @brief Parses the first two bytes of my_rx_buffer[] into a charge_status struct and returns it via argument.
     * @details Meant to be called after calling readWithCommand(CMD_CODE_CHG_STATUS, CMD_LENGTH_CHG_STATUS)
     */
    void parseChargeStatus(charge_status *status);
#endif
    /**
     * @brief Takes in a twos complement number that's length bits and converts it to a 16 bit twos complement number
     * @details Slightly modified version of this https://www.codeproject.com/Tips/1079637/Twos-Complement-for-Unusual-Integer-Sizes
     */
    int16_t UpscaleTwosComplement(int16_t value, size_t length);

    /**
     * @brief Zeros my_rx_buffer
     */
    void clearRXBuffer(void);

    bool readFromWrapper(uint8_t len);

    bool parse_status_word(buffer_data *status_bytes, PMBusStatus *status);


};

#endif // RPB_1600_H