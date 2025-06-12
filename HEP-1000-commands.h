/**
 * This file contains #defines for all of the commands supported by the RPB-1600
 * It also includes data length and N values for decoding/encoding values
 * See the PMBus 1.1 specifications "Linear data" section for more info
 */

#define TRIM_UPPER_BOUND 25.0
#define TRIM_LOWER_BOUND -50.0

// Command Codes
#define CMD_CODE_OPERATION 0x01
#define CMD_CODE_ON_OFF_CONFIG 0x02
#define CMD_CODE_CAPABILITY 0x19
#define CMD_CODE_VOUT_MODE 0x20
#define CMD_CODE_VOUT_COMMAND 0x21
#define CMD_CODE_VOUT_TRIM 0x22
#define CMD_CODE_IOUT_OC_FAULT_LIMIT 0x46
#define CMD_CODE_IOUT_OC_FAULT_RESPONSE 0x47
#define CMD_CODE_STATUS_WORD 0x79
#define CMD_CODE_STATUS_VOUT 0x7A
#define CMD_CODE_STATUS_IOUT 0x7B
#define CMD_CODE_STATUS_INPUT 0x7C
#define CMD_CODE_STATUS_TEMPERATURE 0x7E
#define CMD_CODE_STATUS_CML 0x7E
#define CMD_CODE_STATUS_MFR_SPECIFIC 0x80
#define CMD_CODE_STATUS_FANS_1_2 0x81
#define CMD_CODE_READ_VIN 0x88
#define CMD_CODE_READ_VOUT 0x8B
#define CMD_CODE_READ_IOUT 0x8C
#define CMD_CODE_READ_FAN_SPEED_1 0x90
#define CMD_CODE_READ_FAN_SPEED_2 0x91
#define CMD_CODE_PMBUS_REVISION 0x98
#define CMD_CODE_MFR_ID 0x99
#define CMD_CODE_MFR_MODEL 0x9A
#define CMD_CODE_MFR_REVISION 0x9B
#define CMD_CODE_MFR_LOCATION 0x9C
#define CMD_CODE_MFR_DATE 0x9D
#define CMD_CODE_MFR_SERIAL 0x9E

// Charing Curve config command codes
#define CMD_CODE_CURVE_CC 0xB0
#define CMD_CODE_CURVE_CV 0xB1
#define CMD_CODE_CURVE_FV 0xB2
#define CMD_CODE_CURVE_TC 0xB3
#define CMD_CODE_CURVE_CONFIG 0xB4
#define CMD_CODE_CURVE_CC_TIMEOUT 0xB5
#define CMD_CODE_CURVE_CV_TIMEOUT 0xB6
#define CMD_CODE_CURVE_FLOAT_TIMEOUT 0xB7
#define CMD_CODE_CHG_STATUS 0xB8

// System config command codes
#define CMD_CODE_SYSTEM_CONFIG 0xBE
#define CMD_CODE_SYSTEM_STATUS 0xBF

// Command lengths in bytes
#define CMD_LENGTH_OPERATION 1
#define CMD_LENGTH_ON_OFF_CONFIG 1
#define CMD_LENGTH_CAPABILITY 1
#define CMD_LENGTH_VOUT_MODE 1
#define CMD_LENGTH_VOUT_COMMAND 2
#define CMD_LENGTH_VOUT_TRIM 2
#define CMD_LENGTH_IOUT_OC_FAULT_LIMIT 2
#define CMD_LENGTH_IOUT_OC_FAULT_RESPONSE 1
#define CMD_LENGTH_STATUS_WORD 2
#define CMD_LENGTH_STATUS_VOUT 1
#define CMD_LENGTH_STATUS_IOUT 1
#define CMD_LENGTH_STATUS_INPUT 1
#define CMD_LENGTH_STATUS_TEMPERATURE 1
#define CMD_LENGTH_STATUS_CML 1
#define CMD_LENGTH_STATUS_MFR_SPECIFIC 1
#define CMD_LENGTH_STATUS_FANS_1_2 1
#define CMD_LENGTH_READ_VIN 2
#define CMD_LENGTH_READ_VOUT 2
#define CMD_LENGTH_READ_IOUT 2
#define CMD_LENGTH_READ_FAN_SPEED_1 2
#define CMD_LENGTH_READ_FAN_SPEED_2 2
#define CMD_LENGTH_PMBUS_REVISION 1
#define CMD_LENGTH_MFR_ID 12
#define CMD_LENGTH_MFR_MODEL 12
#define CMD_LENGTH_MFR_REVISION 6
#define CMD_LENGTH_MFR_LOCATION 3
#define CMD_LENGTH_MFR_DATE 6
#define CMD_LENGTH_MFR_SERIAL 12

// Charing Curve config command lengths in bytes
#define CMD_LENGTH_CURVE_CC 2
#define CMD_LENGTH_CURVE_CV 2
#define CMD_LENGTH_CURVE_FV 2
#define CMD_LENGTH_CURVE_TC 2
#define CMD_LENGTH_CURVE_CONFIG 2
#define CMD_LENGTH_CURVE_CC_TIMEOUT 2
#define CMD_LENGTH_CURVE_CV_TIMEOUT 2
#define CMD_LENGTH_CURVE_FLOAT_TIMEOUT 2
#define CMD_LENGTH_CHG_STATUS 2

// System config command lengths in bytes
#define CMD_LENGTH_SYSTEM_CONFIG 2
#define CMD_LENGTH_SYSTEM_STATUS 2

// Linear data N values
#define CMD_N_VALUE_VOUT_MODE -7
#define CMD_N_VALUE_VOUT_COMMAND -7
#define CMD_N_VALUE_VOUT_TRIM -7
#define CMD_N_VALUE_IOUT_OC_FAULT_LIMIT -6
#define CMD_N_VALUE_READ_VIN -1
#define CMD_N_VALUE_READ_VOUT -7
#define CMD_N_VALUE_READ_IOUT -6
//#define CMD_N_VALUE_READ_FAN_SPEED_1 5
//#define CMD_N_VALUE_READ_FAN_SPEED_2 5
#define CMD_N_VALUE_CURVE_CC -6
#define CMD_N_VALUE_CURVE_CV -7
#define CMD_N_VALUE_CURVE_FV -7
#define CMD_N_VALUE_CURVE_TC -6
#define CMD_N_VALUE_CURVE_CC_TIMEOUT 0
#define CMD_N_VALUE_CURVE_CV_TIMEOUT 0
#define CMD_N_VALUE_CURVE_FLOAT_TIMEOUT 0

// System Config High Byte bits
#define SYSTEM_CONFIG_EEP_OFF 2
#define SYSTEM_CONFIG_EEP_CONFIG 0

// System Config Low Byte bits
#define SYSTEM_CONFIG_OPERATION_INIT 1
#define SYSTEM_CONFIG_PM_CTRL 0

// System Config High Byte bitmask
#define SYSTEM_CONFIG_EEP_OFF_MASK 1
#define SYSTEM_CONFIG_EEP_CONFIG_MASK 3

// System Config Low Byte bitmask
#define SYSTEM_CONFIG_OPERATION_INIT_MASK 3
#define SYSTEM_CONFIG_PM_CTRL_MASK 1

// STATUS_BYTE (bits 0–7 of STATUS_WORD)
#define STATUS_WORD_BIT_7_BUSY                7
#define STATUS_WORD_BIT_6_OFF                 6
#define STATUS_WORD_BIT_5_VOUT_OV             5
#define STATUS_WORD_BIT_4_IOUT_OC             4
#define STATUS_WORD_BIT_3_VIN_UV              3
#define STATUS_WORD_BIT_2_TEMPERATURE         2
#define STATUS_WORD_BIT_1_CML                 1
#define STATUS_WORD_BIT_0_NONE_OF_THE_ABOVE   0

// Upper byte of STATUS_WORD (bits 8–15)
#define STATUS_WORD_BIT_15_VOUT               15
#define STATUS_WORD_BIT_14_IOUT               14
#define STATUS_WORD_BIT_13_INPUT              13
#define STATUS_WORD_BIT_12_MFR_SPECIFIC       12
#define STATUS_WORD_BIT_11_POWER_GOOD_N       11
#define STATUS_WORD_BIT_10_FANS               10
#define STATUS_WORD_BIT_9_OTHER               9
#define STATUS_WORD_BIT_8_UNKNOWN             8

#define STATUS_WORD_BIT_MASK 1

// Operation Byte bits
#define OPERATION_ON_OFF 7
#define OPERATION_OFF_TYPE 6 // Only active when 7 is low (PSU is turned off). 0 is immediate, 1 is soft
#define OPERATION_MARGIN 4 // 01 watches for under- conditions and 10 watches for over-
#define OPERATION_FAULT_ACTION 2 // 01 ignores faults, 10 acts on fault conditions

// Operation Byte bit mask
#define OPERATION_ON_OFF_MASK 1
#define OPERATION_OFF_TYPE_MASK 1
#define OPERATION_MARGIN_MASK 3
#define OPERATION_FAULT_ACTION_MASK 3

