#pragma once

#include "i2c.h"

constexpr uint8_t INA219_I2C_ADDRESS1 = 0x40;
constexpr uint8_t INA219_I2C_ADDRESS2 = 0x41;
constexpr uint8_t INA219_I2C_ADDRESS3 = 0x44;
constexpr uint8_t INA219_I2C_ADDRESS4 = 0x45;

// Register Configuration
constexpr uint8_t INA219_REG_CONFIG = 0x00;
constexpr uint16_t INA219_CONFIG_RESET = 0x8000;
constexpr uint16_t INA219_CONFIG_BUSVOLTAGERANGE_MASK = 0x2000;

// Shunt Voltage Register
constexpr uint8_t INA219_REG_SHUNTVOLTAGE = 0x01;
// Bus Voltage Register
constexpr uint8_t INA219_REG_BUSVOLTAGE = 0x02;
// Power Register
constexpr uint8_t INA219_REG_POWER = 0x03;
// Current Register
constexpr uint8_t INA219_REG_CURRENT = 0x04;
// Register Calibration
constexpr uint8_t INA219_REG_CALIBRATION = 0x05;

enum class Ina219_Status : uint8_t{
    OK,
    InitError,
    WriteRegError,
    ReadRegError
};

enum class Ina219BusVolRange : uint8_t {
    VolRange_16V = 0,
    VolRange_32V = 1
};

enum class Ina219PGABits : uint8_t {
    PGABits_1 = 0,
    PGABits_2 = 1,
    PGABits_4 = 2,
    PGABits_8 = 3
};

enum class Ina219AdcBits : uint8_t {
    AdcBits_9 = 0,
    AdcBits_10 = 1,
    AdcBits_11 = 2,
    AdcBits_12 = 3
};

enum class Ina219AdcSample : uint8_t {
    AdcSample_1 = 0,
    AdcSample_2 = 1,
    AdcSample_4 = 2,
    AdcSample_8 = 3,
    AdcSample_16 = 4,
    AdcSample_32 = 5,
    AdcSample_64 = 6,
    AdcSample_128 = 7
};

enum class InaMode : uint8_t {
    eIna219PowerDown = 0,
    eIna219SVolTrig = 1,
    eIna219BVolTrig = 2,
    eIna219SAndBVolTrig = 3,
    eIna219AdcOff = 4,
    eIna219SVolCon = 5,
    eIna219BVolCon = 6,
    eIna219SAndBVolCon = 7
};

class DFRobot_INA219 {
public:
	DFRobot_INA219(I2C_HandleTypeDef* hi2c, uint8_t addr)
		: hi2c_{hi2c}, addr_{addr} {}


	bool init();
	void reset();

	float getBusVoltage_V();
	float getShuntVoltage_mV();
	float getCurrent_mA();
	float getPower_mW();

	void setBRNG(Ina219BusVolRange volRange);
	void setPGA(Ina219PGABits bits);
	void setBADC(Ina219AdcBits bits, Ina219AdcSample sample);
	void setSADC(Ina219AdcBits bits, Ina219AdcSample sample);
	void setMode(InaMode mode);

	void linearCalibrate(float ina219Reading_mA, float extMeterReading_mA);

private:
	uint16_t readReg(uint8_t reg);
	void writeReg(uint8_t reg, uint16_t value);

	I2C_HandleTypeDef* hi2c_;
	uint8_t addr_;

	Ina219_Status lastOperateStatus_;

	uint16_t calValue_;
};
