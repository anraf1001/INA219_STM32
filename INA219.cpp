#include "INA219.hpp"

#include "main.h"

bool DFRobot_INA219::init() {
	lastOperateStatus_ = Ina219_Status::InitError;
	if (HAL_OK == HAL_I2C_IsDeviceReady(hi2c_, addr_ << 1, 3, 5)) {
		setBRNG(Ina219BusVolRange::VolRange_32V);
		setPGA(Ina219PGABits::PGABits_8);
		setBADC(Ina219AdcBits::AdcBits_12, Ina219AdcSample::AdcSample_8);
		setSADC(Ina219AdcBits::AdcBits_12, Ina219AdcSample::AdcSample_8);
		setMode(InaMode::eIna219SAndBVolCon);
		calValue_ = 4096;
		writeReg(INA219_REG_CALIBRATION, calValue_);
		lastOperateStatus_ = Ina219_Status::OK;
		return true;
	}

	return false;
}

void DFRobot_INA219::linearCalibrate(float ina219Reading_mA, float extMeterReading_mA) {
    calValue_ = (uint16_t)((extMeterReading_mA / ina219Reading_mA) * calValue_) & 0xFFFE;
    writeReg(INA219_REG_CALIBRATION, calValue_);
}

float DFRobot_INA219::getBusVoltage_V() {
    return (float) (readReg(INA219_REG_BUSVOLTAGE) >> 1) * 0.001;
}

float DFRobot_INA219::getShuntVoltage_mV() {
    return (float) readReg(INA219_REG_SHUNTVOLTAGE) * 0.01;
}

float DFRobot_INA219::getCurrent_mA() {
    return (float) readReg(INA219_REG_CURRENT);
}

float DFRobot_INA219::getPower_mW() {
	return (float) readReg(INA219_REG_POWER) * 20;
}

void DFRobot_INA219::reset() {
	writeReg(INA219_REG_CONFIG, INA219_CONFIG_RESET);
}

// Sets Bus Voltage Range (default value is 32V)
void DFRobot_INA219::setBRNG(Ina219BusVolRange volRange) {
	uint16_t conf = readReg(INA219_REG_CONFIG);
	conf &= ~((uint16_t) 1 << 13);
	conf |= static_cast<uint16_t>(volRange) << 13;
	writeReg(INA219_REG_CONFIG, conf);
}

// Sets PGA gain and range (default value is 320mV)
void DFRobot_INA219::setPGA(Ina219PGABits bits) {
	uint16_t conf = readReg(INA219_REG_CONFIG);
	conf &= ~((uint16_t) 0x03 << 11);
	conf |= static_cast<uint16_t>(bits) << 11;
	writeReg(INA219_REG_CONFIG, conf);
}

// These bits adjust the Bus ADC resolution (9-, 10-, 11-, or 12-bit)
// or set the number of samples used when averaging results for the Bus Voltage Register
void DFRobot_INA219::setBADC(Ina219AdcBits bits, Ina219AdcSample sample) {
	uint16_t value;

	if (bits < Ina219AdcBits::AdcBits_12) {
		value = static_cast<uint16_t>(bits);
	} else {
		value = 0x08 | static_cast<uint16_t>(sample);
	}

	uint16_t conf = readReg(INA219_REG_CONFIG);
	conf &= ~((uint16_t) 0x0f << 7);
	conf |= static_cast<uint16_t>(value) << 7;
	writeReg(INA219_REG_CONFIG, conf);
}

// These bits adjust the Shunt ADC resolution (9-, 10-, 11-, or 12-bit)
// or set the number of samples used when averaging results for the Shunt Voltage Register
void DFRobot_INA219::setSADC(Ina219AdcBits bits, Ina219AdcSample sample) {
	uint16_t value;

	if (bits < Ina219AdcBits::AdcBits_12) {
		value = static_cast<uint16_t>(bits);
	} else {
		value = 0x08 | static_cast<uint16_t>(sample);
	}

	uint16_t conf = readReg(INA219_REG_CONFIG);
	conf &= ~((uint16_t) 0x0f << 3);
	conf |= static_cast<uint16_t>(value) << 3;
	writeReg(INA219_REG_CONFIG, conf);
}

void DFRobot_INA219::setMode(InaMode mode) {
    uint16_t conf = readReg(INA219_REG_CONFIG);
    conf &= ~((uint16_t) 0x07);
    conf |= static_cast<uint16_t>(mode);
    writeReg(INA219_REG_CONFIG, conf);
}

uint16_t DFRobot_INA219::readReg(uint8_t reg) {
	uint8_t buffer[2] = {};

	if (HAL_OK == HAL_I2C_Mem_Read(hi2c_, addr_ << 1, reg, 1, buffer, 2, 100)) {
		lastOperateStatus_ = Ina219_Status::OK;
	} else {
		lastOperateStatus_ = Ina219_Status::ReadRegError;
	}

	return (buffer[0] << 8) | buffer[1];
}

void DFRobot_INA219::writeReg(uint8_t reg, uint16_t value) {
	uint8_t buffer[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};

	if (HAL_OK == HAL_I2C_Mem_Write(hi2c_, addr_ << 1, reg, 1, buffer, 2, 100)) {
		lastOperateStatus_ = Ina219_Status::OK;
	} else {
		lastOperateStatus_ = Ina219_Status::WriteRegError;
	}
}
