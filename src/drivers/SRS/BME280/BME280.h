/******************************************************************************
BME280.h
SRS BME280 Arduino Driver
UT2UH
March 07, 2021

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.

******************************************************************************/

// Test derived class for base class SparkFunIMU
#ifndef __BME280_H__
#define __BME280_H__

#include "Arduino.h"
#include <Wire.h>

// Model allows us to specify which digital BMx280 sensor is detected
enum Chip { BME_280, BMP_280 };

//  The valid addresses for BME280 sensors
const uint8_t adrBme0x76 = 0x76;	//if jumper closed
const uint8_t adrBme0x77 = 0x77;

//Register names:
#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2
#define BME280_STAT_REG					0xF3
#define BME280_CTRL_MEAS_REG			0xF4
#define BME280_CONFIG_REG				0xF5
#define BME280_PRESSURE_MSB_REG			0xF7
#define BME280_PRESSURE_LSB_REG			0xF8
#define BME280_PRESSURE_XLSB_REG		0xF9
#define BME280_TEMPERATURE_MSB_REG		0xFA
#define BME280_TEMPERATURE_LSB_REG		0xFB
#define BME280_TEMPERATURE_XLSB_REG		0xFC
#define BME280_HUMIDITY_MSB_REG			0xFD
#define BME280_HUMIDITY_LSB_REG			0xFE

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct SensorCalibration {
  public:
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t  dig_H6;	
};

struct FloatCalibration {
  public:
	float t1;
	float t2;
	float t3;
	float p1;
	float p2;
	float p3;
	float p4;
	float p5;
	float p6;
	float p7;
	float p8;
	float p9;
	float h1;
	float h2;
	float h3;
	float h4;
	float h5;
	float h6;
};

// @brief  sampling rates
enum sensor_sampling {
	SAMPLING_NONE = 0b000,
	SAMPLING_X1 = 0b001,
	SAMPLING_X2 = 0b010,
	SAMPLING_X4 = 0b011,
	SAMPLING_X8 = 0b100,
	SAMPLING_X16 = 0b101
};

//  power modes
enum sensor_mode {
	MODE_SLEEP = 0b00,
	MODE_FORCED = 0b01,
	MODE_NORMAL = 0b11
};

// filter values
enum sensor_filter {
	FILTER_OFF = 0b000,
	FILTER_X2 = 0b001,
	FILTER_X4 = 0b010,
	FILTER_X8 = 0b011,
	FILTER_X16 = 0b100
};

//  standby duration in ms
enum standby_duration {
	STANDBY_MS_0_5 = 0b000,
	STANDBY_MS_10 = 0b110,
	STANDBY_MS_20 = 0b111,
	STANDBY_MS_62_5 = 0b001,
	STANDBY_MS_125 = 0b010,
	STANDBY_MS_250 = 0b011,
	STANDBY_MS_500 = 0b100,
	STANDBY_MS_1000 = 0b101
};


//Class BME280_SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out BME280_SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)
//
struct BME280_SensorSettings {
  public:
	sensor_mode runMode;
	standby_duration tStandby;
	sensor_filter filter;
	sensor_sampling tempOverSample;
	sensor_sampling pressOverSample;
	sensor_sampling humidOverSample;
    float tempCorrection; // correction of temperature - added to the result
};

class BME280 {
  public:
    //settings
    BME280_SensorSettings settings;
	int32_t t_fine;
	
	//Constructor generates default BME280_SensorSettings.
	//(over-ride after construction if desired)
    BME280(const uint8_t addr = adrBme0x76);
	
	//Call to apply BME280_SensorSettings.
	//This also gets the SensorCalibration constants
    bool begin(void);
	void reset(void);

	sensor_mode getMode(void); //Get the current mode: sleep, forced, or normal
	void setMode(sensor_mode mode); //Set the current mode
	void setSampling(sensor_mode mode,
						sensor_sampling tempSampling,
						sensor_sampling pressSampling,
						sensor_sampling humSampling,
						sensor_filter filter,
						standby_duration duration);
	void setTempOverSample(sensor_sampling tempSampling); //Set the temperature sample mode
	void setPressureOverSample(sensor_sampling pressSampling); //Set the pressure sample mode
	void setHumidityOverSample(sensor_sampling humSampling); //Set the humidity sample mode
	void setStandbyDuration(standby_duration duration); //Set the standby time between measurements
	void setFilter(sensor_filter filter); //Set the filter

	bool isMeasuring(void); //Returns true while the device is taking measurement

	bool  updateData(void);
    float getPressure(void);
	float getHumidity(void);
    float getTempC(void);

	float finePressure(void);
	float fineHumidity(void);
	float fineTempC(void);
	float getVirtTemperature(void);

    void  setTemperatureCorrection(float corr);
	float getTemperatureCompensation(void);		//Ada
	void  setTemperatureCompensation(float);	//Ada

	float GetSpeedOfSound(float temperature, float humidity);

	void  setReferencePressure(float refPressure); //Allows user to set local sea level reference pressure
	float getReferencePressure();
	
	float readAltitudeMeters(void);
	float readAltitude(float seaLevel);	//Ada
	float seaLevelForAltitude(float altitude, float pressure);	//Ada
private:
	// The sensor model number
	uint8_t chipID;
	// The address of this device
	uint8_t addr;
	// add to compensate temp readings and in turn
	// to pressure and humidity readings
  	int32_t t_fine_adjust = 0; 
	// Default but is changeable
	float referencePressure = 101325.0;

    const float temperature_min = -40;
    const float temperature_max = 85;
	SensorCalibration iCal;
	FloatCalibration fcal;

	float a[16] = 
	{
		331.5024,
		0.603055,
		-0.000528,
		51.471935,
		0.1495874,
		-0.000782,
		-1.82e-7,
		3.73e-8,
		-2.93e-10,
		-85.20931,
		-0.228525,
		5.91e-5,
		-2.835149,
		-2.15e-13,
		29.179762,
		0.000486
	};
	float p = 101000;

    uint8_t bme280(uint8_t reg);
    void bme280(uint8_t reg, uint8_t value);
    void bme280(uint8_t reg, uint8_t size, uint8_t *data);

};

#endif  // End of __BME280_H__