/*
	UT2UH LTC2943 Arduino Library
	Based on the excellent Macro Yau LTC2942 Library
	https://github.com/MacroYau/LTC2942-Arduino-Library
	Not finished - work in progress!
*/


#include "LTC2943.h"

LTC2943::LTC2943(uint8_t rSense) {
	_rSense = rSense;
	_prescalerM = 0xFF;
	_batteryCapacity = 5500; // Default value when M = 128
	_chargeBefore = 0;
	_usageFromStart = 0;
	meanUsagePerHour = 0;
	availableHours = 0;
}

bool LTC2943::begin() {
	// Checks device ID
	uint8_t chipID = getStatus() >> A_CHIP_ID_OFFSET;
	if (chipID != CHIP_ID_LTC2942) {
		return false;
	}
	return true;
}

uint8_t LTC2943::ltc2943(uint8_t address) {
	uint8_t value = 0;

	Wire.beginTransmission(LTC2943_ADDRESS);
	Wire.write(address);
	Wire.endTransmission(false);

	Wire.requestFrom(LTC2943_ADDRESS, 1);
	value = Wire.read();
	Wire.endTransmission();

	return value;
}

bool LTC2943::ltc2943(uint8_t address, uint8_t value) {
	Wire.beginTransmission(LTC2943_ADDRESS);
	Wire.write(address);
	Wire.write(value);
	return (Wire.endTransmission() == 0);
}

uint16_t LTC2943::ltc2943word(uint8_t msbAddress) {
	uint16_t value = 0;
	uint8_t msb = 0;
	uint8_t lsb = 0;

	Wire.beginTransmission(LTC2943_ADDRESS);
	Wire.write(msbAddress);
	Wire.endTransmission(false);

	Wire.requestFrom(LTC2943_ADDRESS, 2);
	msb = Wire.read();
	lsb = Wire.read();
	Wire.endTransmission();
	value = (msb << 8) | lsb;

	return value;
}

bool LTC2943::ltc2943word(uint8_t msbAddress, uint16_t value) {
	Wire.beginTransmission(LTC2943_ADDRESS);
	Wire.write(msbAddress);
	Wire.write((uint8_t) (value >> 8));
	Wire.write((uint8_t) value);
	return (Wire.endTransmission() == 0);
}

void LTC2943::resetAlert() {
	Wire.requestFrom(LTC2943_ARA_ADR, 1);
	Wire.read();
	Wire.endTransmission();
}

bool LTC2943::onPowerLost() {
	///;Serial.print("A "); Serial.print(ltc2943(LTC2943_A_STATUS), HEX); 
	///;Serial.print(" B "); Serial.print(ltc2943(LTC2943_A_STATUS), HEX);
	///;Serial.print(" C "); Serial.print(ltc2943word(LTC2943_C_ACC_CHG_MSB), HEX);
	///;Serial.print(" TH "); Serial.print(ltc2943word(LTC2943_E_CHG_THR_H_MSB), HEX);
	///;Serial.print(" TL "); Serial.println(ltc2943word(LTC2943_G_CHG_THR_L_MSB), HEX);

	if( (ltc2943(LTC2943_A_STATUS) == 0x00) &&
	    ((ltc2943(LTC2943_B_CONTROL) & 0x3E) == 0x3C ) &&
		(ltc2943(LTC2943_C_ACC_CHG_MSB) == 0x7F ) &&		
		(ltc2943word(LTC2943_E_CHG_THR_H_MSB) == 0xFFFF ) &&
		(ltc2943word(LTC2943_G_CHG_THR_L_MSB) == 0x0000 )) {
			return true;
		}
	return false;
}

void LTC2943::startMeasurement() {
	uint8_t value = ltc2943(LTC2943_B_CONTROL);
	value &= SHUTDOWN_MASK;
	ltc2943(LTC2943_B_CONTROL, value);
}

void LTC2943::stopMeasurement() {
	uint8_t value = ltc2943(LTC2943_B_CONTROL);
	value |= 1;
	ltc2943(LTC2943_B_CONTROL, value);
}

uint8_t LTC2943::getStatus() {
	return ltc2943(LTC2943_A_STATUS);
}

uint16_t LTC2943::getRawAccumulatedCharge() {
	if (_prescalerM == 0xFF) {
		// Needs to obtain M from register B
		uint8_t value = ltc2943(LTC2943_B_CONTROL);
		value &= ~PRESCALER_M_MASK;
		_prescalerM = value >> B_PRESCALER_M_OFFSET;
		_prescalerM = 1 << _prescalerM;
	}
	uint16_t acr = ltc2943word(LTC2943_C_ACC_CHG_MSB);
	return acr;
}

uint16_t LTC2943::getRemainingCapacity() {
	uint16_t acr = getRawAccumulatedCharge();
	//float fullRange = 65536 * ((float) _prescalerM / 128) * 0.085;
	//float offset = fullRange - _batteryCapacity;
	//charge in mAh, multiplier of 50 is split to 5 and 10 to prevent unsigned long overflow
    return (uint16_t)(((uint32_t)acr * _num / _den) - _offset);	
}

/*
float LTC2943::getRemainingCapacity() {
	uint16_t acr = getRawAccumulatedCharge();
	float fullRange = 65536 * ((float) _prescalerM / 128) * 0.085;
	float offset = fullRange - _batteryCapacity;
	return (acr * ((float) _prescalerM / 128) * 0.085 * ((float) 50 / _rSense)) - offset; // mAh
}
*/

uint16_t LTC2943::getRemainingTime(uint8_t idx) {
	uint16_t chargeNow = getRawAccumulatedCharge();
	if(idx == 0){
		_chargeBefore = chargeNow;
	} else {
		uint16_t usagePerInterval = _chargeBefore - chargeNow;
		_chargeBefore  = chargeNow;
		uint16_t usagePerHour = (60 / intervalM[idx - 1]) * usagePerInterval;
		_usageFromStart += usagePerHour;
		meanUsagePerHour = _usageFromStart / idx;
		availableHours = chargeNow / meanUsagePerHour;
	}
	return availableHours;
}

/*
 *  Note:
 *  1. Datasheet conversion formula divide by 65535, in this library we divide by 65536 (>> 16) to reduce computational load 
 *     this is acceptable as the difference is much lower than the resolution of LTC2942 voltage measurement (78mV)
 *  2. Return is in unsigned short and mV to prevent usage of float datatype, the resolution of LTC2942 voltage measurement (78mV), 
 *     floating point offset is acceptable as it is lower than the resolution of LTC2942 voltage measurement (78mV)
 */
uint16_t LTC2943::getVoltage(bool oneShot) {
	if (oneShot) {
		setADCMode(ADC_MODE_MANUAL_VOLTAGE);
		//for(uint16_t i=0; i<4000; ++i);  // Small delay to wait for the converstion to complete - Ideally it should be 10ms
		delay(10);
	}
	uint16_t value = ltc2943word(LTC2943_I_VOLTAGE_MSB);
	uint32_t vBat = ((uint32_t)value * LTC2943_FULLSCALE_VOLTAGE);	// FULLSCALE_VOLTAGE is in mV, to avoid using float datatype
    vBat >>= 16;
	return (uint16_t)vBat;	//mV
}

/*
 *  Note:
 *  1. Datasheet conversion formula divide by 65535, in this library we divide by 65536 (>> 16) to reduce computational load 
 *     this is acceptable as the difference is much lower than the resolution of LTC2942 voltage measurement (78mV)
 *  2. Return is in unsigned short and mV to prevent usage of float datatype, the resolution of LTC2942 voltage measurement (78mV), 
 *     floating point offset is acceptable as it is lower than the resolution of LTC2942 voltage measurement (78mV)
 */
uint16_t LTC2943::getCurrent(bool oneShot) {
	if (oneShot) {
		setADCMode(ADC_MODE_MANUAL_VOLTAGE);
		//for(uint16_t i=0; i<4000; ++i);  // Small delay to wait for the converstion to complete - Ideally it should be 10ms
		delay(10);
	}
	uint16_t value = ltc2943word(LTC2943_I_VOLTAGE_MSB);
	uint32_t vBat = ((uint32_t)value * LTC2943_FULLSCALE_VOLTAGE);	// FULLSCALE_VOLTAGE is in mV, to avoid using float datatype
    vBat >>= 16;
	return (uint16_t)vBat;	//mV
}

/*
 *  Note:
 *  1. Datasheet conversion formula divide by 65535, in this library we divide by 65536 (>> 16) to reduce computational load 
 *     this is acceptable as the difference is much lower than the resolution of LTC2942 temperature measurement (3 Celcius)
 *  2. Return is in short to prevent usage of float datatype, floating point offset is acceptable as it is lower than the resolution of LTC2942 voltage measurement (3 Celcius).
 *     Unit of 0.01 Celcius
 */
int16_t LTC2943::getTemperature(bool oneShot) {
	if (oneShot) {
		setADCMode(ADC_MODE_MANUAL_TEMP);
		//for(uint16_t i=0; i<4000; ++i);  // Small delay to wait for the converstion to complete - Ideally it should be 10ms
		delay(10);
	}
	uint16_t value = ltc2943(LTC2943_U_TEMP_MSB);	
	uint32_t tBat = ((uint32_t)value * LTC2943_FULLSCALE_TEMPERATURE);
    tBat >>= 16;
    tBat -= 2731;  // Convert temperature from kelvin to degree celsius	
	return (int16_t)tBat;
}

void LTC2943::setADCMode(uint8_t mode) {
	if (mode > 0b11) {
		return;
	}

	uint8_t value = ltc2943(LTC2943_B_CONTROL);
	value &= ADC_MODE_MASK;
	value |= (mode << B_ADC_MODE_OFFSET);
	ltc2943(LTC2943_B_CONTROL, value);
}

void LTC2943::setPrescalerM(uint8_t m) {
	if (m < 1 || m > 128) {
		return;
	}

	// Updates instance variable to avoid unnecessary access to register
	_prescalerM = m;
	m = findExponentOfPowerOfTwo(m);

	uint8_t value = ltc2943(LTC2943_B_CONTROL);
	value &= PRESCALER_M_MASK;
	value |= (m << B_PRESCALER_M_OFFSET);
	ltc2943(LTC2943_B_CONTROL, value);
}

/*
// Credit to https://github.com/DelfiSpace/LTC2942/blob/master/LTC2942.cpp
void LTC2943::setBatteryCapacity(uint16_t mAh) {
	_batteryCapacity = mAh;		
    unsigned int k, a;
    uint8_t m = 7;
    for(k = 128; k > 1; k = k / 2) {
        a = 278524 * k / _rSense / 128;
        if (a < (2 * mAh)) {
            break;
        }
        m--;
    }

    _num = 87 * 50 * k;
    _den = 128 * _rSense;
    _offset = (64 * _num / _den) - mAh;
	setPrescalerM(m);
}
*/

void LTC2943::setBatteryCapacity(uint16_t mAh) {
	_batteryCapacity = mAh;
	float q = (float) mAh / 1000;
	uint8_t m = 23 * q;
	if (_rSense != 50) {
		m *= ((float) _rSense / 50);
	}
	if (m > 128) {
		m = 128;
	}
	m = roundUpToPowerOfTwo(m);
    _num = 87 * 50 * m;
    _den = 128 * _rSense;
    _offset = (64 * _num / _den) - mAh;	
	setPrescalerM(m);
}

void LTC2943::setBatteryToFull() {
	ltc2943word(LTC2943_C_ACC_CHG_MSB, 0xFFFF);
}

void LTC2943::setRawAccumulatedCharge(uint16_t charge) {
	ltc2943word(LTC2943_C_ACC_CHG_MSB, charge);
}

void LTC2943::setChargeThresholds(uint16_t high, uint16_t low) {
	ltc2943word(LTC2943_E_CHG_THR_H_MSB, high);
	ltc2943word(LTC2943_G_CHG_THR_L_MSB, low);
}

void LTC2943::setVoltageThresholds(uint16_t high, uint16_t low) {
	ltc2943(LTC2943_K_VOLTAGE_THR_H_MSB, (high / 23.4375));
	ltc2943(LTC2943_N_VOLTAGE_THR_L_LSB, (low / 23.4375));
}

void LTC2943::setCurrentThresholds(uint16_t high, uint16_t low) {
	ltc2943(LTC2943_Q_CURRENT_THR_H_MSB, (high / 23.4375)); 
	ltc2943(LTC2943_S_CURRENT_THR_L_MSB, (low / 23.4375)); 
}

void LTC2943::setTemperatureThresholds(int16_t high, int16_t low) {
	ltc2943(LTC2943_U_TEMP_MSB, (uint8_t) ((high + 2731) / 23.4375));
	ltc2943(LTC2943_V_TEMP_LSB, (uint8_t) ((low + 2731) / 23.4375));
}

void LTC2943::configureALCC(uint8_t mode) {
	if (mode >= ALCC_MODE_NOT_ALLOWED) {
		return;
	}

	uint8_t value = ltc2943(LTC2943_B_CONTROL);
	value &= ALCC_CONFIG_MASK;
	value |= (mode << B_ALCC_CONFIG_OFFSET);
	ltc2943(LTC2943_B_CONTROL, value);
}

uint8_t LTC2943::findExponentOfPowerOfTwo(uint8_t value) {
	if (value > 64) {
		return 7;
	}

	for (uint8_t i = 0; i < 7; i++) {
		if ((value >> i) & 1) {
			return i;
		}
	}
	return 0;
}

uint8_t LTC2943::roundUpToPowerOfTwo(uint8_t value) {
	// Reference: https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
	value--;
	value |= value >> 1;
	value |= value >> 2;
	value |= value >> 4;
	value++;
	return value;
}