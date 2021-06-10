/*
	UT2UH LTC2943 Arduino Library
	Based on the excellent Macro Yau LTC2942 Library
	https://github.com/MacroYau/LTC2942-Arduino-Library
*/

#ifndef LTC2943_H
#define LTC2943_H

#include "Arduino.h"
#include "Wire.h"

#define LTC2943_ADDRESS			0x64
#define LTC2943_ARA_ADR			0x0C

/* Register Map */

#define LTC2943_A_STATUS			0x00 // Status (R)
#define LTC2943_B_CONTROL			0x01 // Control (R/W)
#define LTC2943_C_ACC_CHG_MSB		0x02 // Accumulated Charge MSB (R/W)
#define LTC2943_D_ACC_CHG_LSB		0x03 // Accumulated Charge LSB (R/W)
#define LTC2943_E_CHG_THR_H_MSB		0x04 // Charge Threshold High MSB (R/W)
#define LTC2943_F_CHG_THR_H_LSB		0x05 // Charge Threshold High LSB (R/W)
#define LTC2943_G_CHG_THR_L_MSB		0x06 // Charge Threshold Low MSB (R/W)
#define LTC2943_H_CHG_THR_L_LSB		0x07 // Charge Threshold Low LSB (R/W)
#define LTC2943_I_VOLTAGE_MSB		0x08 // Voltage MSB (R)
#define LTC2943_J_VOLTAGE_LSB		0x09 // Voltage LSB (R)

#define LTC2943_K_VOLTAGE_THR_H_MSB		0x0A // Voltage Threshold High MSB (R/W)
#define LTC2943_L_VOLTAGE_THR_H_LSB		0x0B // Voltage Threshold High LSB (R/W)
#define LTC2943_M_VOLTAGE_THR_L_MSB		0x0C // Voltage Threshold Low MSB (R/W)
#define LTC2943_N_VOLTAGE_THR_L_LSB		0x0D // Voltage Threshold Low LSB (R/W)

#define LTC2943_O_CURRENT_MSB		0x0E // Current MSB (R)
#define LTC2943_P_CURRENT_LSB		0x0F // Current LSB (R)
#define LTC2943_Q_CURRENT_THR_H_MSB		0x10 // Current Threshold High MSB (R/W)
#define LTC2943_R_CURRENT_THR_H_LSB		0x11 // Current Threshold High LSB (R/W)
#define LTC2943_S_CURRENT_THR_L_MSB		0x12 // Current Threshold Low MSB (R/W)
#define LTC2943_T_CURRENT_THR_L_LSB		0x13 // Current Threshold Low LSB (R/W)

#define LTC2943_U_TEMP_MSB			0x14 // Temperature MSB (R)
#define LTC2943_V_TEMP_LSB			0x15 // Temperature LSB (R)
#define LTC2943_W_TEMP_THR_H		0x16 // Temperature Threshold High (R/W)
#define LTC2943_X_TEMP_THR_L		0x17 // Temperature Threshold Low (R/W)

/* Status Register (A) */

#define A_CHIP_ID_OFFSET		7 // Default 0
#define CHIP_ID_LTC2942			0
#define CHIP_ID_LTC2941			1
// Bits below are cleared after register is read
#define A_ACC_CHG_OF_UF_OFFSET	5 // Default 0
#define A_TEMP_ALERT_OFFSET		4 // Default 0
#define A_CHG_ALERT_H_OFFSET	3 // Default 0
#define A_CHG_ALERT_L_OFFSET	2 // Default 0
#define A_VOLTAGE_ALERT_OFFSET	1 // Default 0
#define A_UV_LOCK_ALERT_OFFSET	0

/* Control Register (B) */

#define B_ADC_MODE_OFFSET		6 // Default [00]
#define ADC_MODE_MASK			0b00111111
#define ADC_MODE_AUTO			0b11
#define ADC_MODE_MANUAL_VOLTAGE	0b10
#define ADC_MODE_MANUAL_TEMP	0b01
#define ADC_MODE_SLEEP			0b00

#define B_PRESCALER_M_OFFSET	3 // Default [111], M = 2^(4 * B[5] + 2 * B[4] + B[3])
#define PRESCALER_M_MASK		0b11000111

#define B_ALCC_CONFIG_OFFSET	1 // Default [10]
#define ALCC_CONFIG_MASK		0b11111001
#define ALCC_MODE_ALERT			0b10
#define ALCC_MODE_CHG_COMPLETE	0b01
#define ALCC_MODE_DISABLED		0b00
#define ALCC_MODE_NOT_ALLOWED	0b11

#define SHUTDOWN_MASK			0b11111110

#define LTC2943_FULLSCALE_VOLTAGE      6000	// Full scale voltage in mv
#define LTC2943_FULLSCALE_TEMPERATURE  6000	// Full scale temp in 0.1 degC


class LTC2943 {
	public:
		LTC2943(uint8_t rSense = 50);	// Sense resistor value in mOhm
		bool begin();
		bool onPowerLost();
		void startMeasurement();
		void stopMeasurement();
		uint8_t getStatus();
		void resetAlert();
		
		uint16_t getRawAccumulatedCharge();
		uint16_t getRemainingCapacity();				// Capacity in mAh
		uint16_t getRemainingTime(uint8_t idx);			// Time in Hours
		uint16_t getVoltage(bool oneShot = true);		// Voltage in mV
		uint16_t getCurrent(bool oneShot = true);		// Voltage in mA
		int16_t  getTemperature(bool oneShot = true);	// Temperature in units of 0.1 Celcius	

		void setADCMode(uint8_t mode);
		void setPrescalerM(uint8_t m);
		void setBatteryCapacity(uint16_t mAh);			// battery capacity in mAh
		void setBatteryToFull();
		void setRawAccumulatedCharge(uint16_t charge);

		void setChargeThresholds(uint16_t high, uint16_t low);
		void setVoltageThresholds(uint16_t high, uint16_t low);     // Voltage in mV
		void setCurrentThresholds(uint16_t high, uint16_t low);     // Voltage in mA
		void setTemperatureThresholds(int16_t high, int16_t low); // Temperature in units of 0.1 Celcius

		void configureALCC(uint8_t mode);
		uint8_t findExponentOfPowerOfTwo(uint8_t value);
		uint8_t roundUpToPowerOfTwo(uint8_t value);

		uint16_t meanUsagePerHour;
		uint16_t availableHours;
		const uint8_t intervalM[5] = {1, 6, 15, 30, 60};

	private:
		uint8_t _rSense;
		uint8_t _prescalerM;
		uint16_t _batteryCapacity;
		uint16_t _chargeBefore;
		uint16_t _usageFromStart;

		uint8_t M;			// prescaler
		uint32_t _num;      // numerator
		uint32_t _den;      // denominator
		uint32_t _offset;   // offset

		uint8_t ltc2943(uint8_t address);
		bool ltc2943(uint8_t address, uint8_t value);
		uint16_t ltc2943word(uint8_t msbAddress);
		bool ltc2943word(uint8_t msbAddress, uint16_t value);
};

#endif
