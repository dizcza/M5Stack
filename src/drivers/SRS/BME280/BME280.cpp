/******************************************************************************
BME280.cpp
SRS BME280 Arduino Driver
UT2UH
March 07, 2021

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "BME280.h"
#include <math.h>

BME280::BME280(const uint8_t addr)
{
	this->addr = addr;
	
	this->settings.runMode = MODE_SLEEP;
	this->settings.tStandby = STANDBY_MS_0_5;
	this->settings.filter = FILTER_OFF;
	this->settings.tempOverSample  = SAMPLING_X1;
	this->settings.pressOverSample = SAMPLING_X1;
	this->settings.humidOverSample = SAMPLING_X1;
	this->settings.tempCorrection = 0.f; // correction of temperature - added to the result
}

//****************************************************************************//
//
//  This uses the stored BME280_SensorSettings
//
//****************************************************************************//
bool BME280::begin()
{
	//Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	//Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
	this->chipID = bme280(BME280_CHIP_ID_REG);
	if(this->chipID != 0x60) // Is this BME280?
		return(false); //This is not BME280 - useless, no RH!

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	this->iCal.dig_T1 = ((uint16_t)((bme280(BME280_DIG_T1_MSB_REG) << 8) + bme280(BME280_DIG_T1_LSB_REG)));
	this->iCal.dig_T2 = ((int16_t)((bme280(BME280_DIG_T2_MSB_REG) << 8) + bme280(BME280_DIG_T2_LSB_REG)));
	this->iCal.dig_T3 = ((int16_t)((bme280(BME280_DIG_T3_MSB_REG) << 8) + bme280(BME280_DIG_T3_LSB_REG)));

	this->iCal.dig_P1 = ((uint16_t)((bme280(BME280_DIG_P1_MSB_REG) << 8) + bme280(BME280_DIG_P1_LSB_REG)));
	this->iCal.dig_P2 = ((int16_t)((bme280(BME280_DIG_P2_MSB_REG) << 8) + bme280(BME280_DIG_P2_LSB_REG)));
	this->iCal.dig_P3 = ((int16_t)((bme280(BME280_DIG_P3_MSB_REG) << 8) + bme280(BME280_DIG_P3_LSB_REG)));
	this->iCal.dig_P4 = ((int16_t)((bme280(BME280_DIG_P4_MSB_REG) << 8) + bme280(BME280_DIG_P4_LSB_REG)));
	this->iCal.dig_P5 = ((int16_t)((bme280(BME280_DIG_P5_MSB_REG) << 8) + bme280(BME280_DIG_P5_LSB_REG)));
	this->iCal.dig_P6 = ((int16_t)((bme280(BME280_DIG_P6_MSB_REG) << 8) + bme280(BME280_DIG_P6_LSB_REG)));
	this->iCal.dig_P7 = ((int16_t)((bme280(BME280_DIG_P7_MSB_REG) << 8) + bme280(BME280_DIG_P7_LSB_REG)));
	this->iCal.dig_P8 = ((int16_t)((bme280(BME280_DIG_P8_MSB_REG) << 8) + bme280(BME280_DIG_P8_LSB_REG)));
	this->iCal.dig_P9 = ((int16_t)((bme280(BME280_DIG_P9_MSB_REG) << 8) + bme280(BME280_DIG_P9_LSB_REG)));

	this->iCal.dig_H1 = ((uint8_t)(bme280(BME280_DIG_H1_REG)));
	this->iCal.dig_H2 = ((int16_t)((bme280(BME280_DIG_H2_MSB_REG) << 8) + bme280(BME280_DIG_H2_LSB_REG)));
	this->iCal.dig_H3 = ((uint8_t)(bme280(BME280_DIG_H3_REG)));
	this->iCal.dig_H4 = ((int16_t)((bme280(BME280_DIG_H4_MSB_REG) << 4) + (bme280(BME280_DIG_H4_LSB_REG) & 0x0F)));
	this->iCal.dig_H5 = ((int16_t)((bme280(BME280_DIG_H5_MSB_REG) << 4) + ((bme280(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	this->iCal.dig_H6 = ((int8_t)bme280(BME280_DIG_H6_REG));

	//Most of the time the sensor will be init with default values
	//But in case user has old/deprecated code, use the settings.x values
	setSampling(this->settings.runMode,			//Default to MODE_SLEEP
				this->settings.tempOverSample,	//Default of 1x oversample
				this->settings.pressOverSample,	//Default of 1x oversample
				this->settings.humidOverSample,	//Default of 1x oversample	
				this->settings.filter,			//Default to filter off
				this->settings.tStandby);		//Default to 0.5ms


	this->fcal.t1 =  this->iCal.dig_T1 * powf(2,  4);
	this->fcal.t2 =  this->iCal.dig_T2 * powf(2, -14);
	this->fcal.t3 =  this->iCal.dig_T3 * powf(2, -34);

	this->fcal.p1 = this->iCal.dig_P1            	  	  * (powf(2,  4) / -100000.0f);
	this->fcal.p2 = this->iCal.dig_P1 * this->iCal.dig_P2 * (powf(2, -31) / -100000.0f);
	this->fcal.p3 = this->iCal.dig_P1 * this->iCal.dig_P3 * (powf(2, -51) / -100000.0f);

	this->fcal.p4 = this->iCal.dig_P4 * powf(2,  4) - powf(2, 20);
	this->fcal.p5 = this->iCal.dig_P5 * powf(2, -14);
	this->fcal.p6 = this->iCal.dig_P6 * powf(2, -31);

	this->fcal.p7 = this->iCal.dig_P7 * powf(2, -4);
	this->fcal.p8 = this->iCal.dig_P8 * powf(2, -19) + 1.0f;
	this->fcal.p9 = this->iCal.dig_P9 * powf(2, -35);

	this->fcal.h1 = this->iCal.dig_H1 * powf(2, -19);
	this->fcal.h2 = this->iCal.dig_H1 * powf(2, -16);
	this->fcal.h3 = this->iCal.dig_H1 * powf(2, -26);

	this->fcal.h4 = this->iCal.dig_H4 * powf(2,  4) ;
	this->fcal.h5 = this->iCal.dig_H5 * powf(2, -14);
	this->fcal.h6 = this->iCal.dig_H6 * powf(2, -26);

	return true;
}

// Single register read and write
uint8_t BME280::bme280(uint8_t reg)
{
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(this->addr, uint8_t(1));
	return Wire.read();
}

void BME280::bme280(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void BME280::bme280(uint8_t reg, uint8_t size, uint8_t* data)
{
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(this->addr, size);
	for (uint8_t i = 0; i < size; i++) data[i] = Wire.read();
}

//Gets the current mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
sensor_mode BME280::getMode()
{
	uint8_t controlData = bme280(BME280_CTRL_MEAS_REG);
	return (sensor_mode)(controlData & 0b00000011); //Clear bits 7 through 2
}

//Set the mode bits in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BME280::setMode(sensor_mode mode)
{
	if(mode > MODE_NORMAL) mode = MODE_SLEEP; //Error check. Default to sleep mode
	
	uint8_t controlData = bme280(BME280_CTRL_MEAS_REG);
	controlData &= 0b11111100; //Clear the mode[1:0] bits
	controlData |= (uint8_t)mode; //Set
	bme280(BME280_CTRL_MEAS_REG, controlData);
}

/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
void BME280::setSampling(sensor_mode mode,
							sensor_sampling tempSampling,
							sensor_sampling pressSampling,
							sensor_sampling humSampling,
							sensor_filter filter,
							standby_duration duration)
{

	// making sure sensor is in sleep mode before setting configuration
	// as it otherwise may be ignored
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to humSampling
	uint8_t controlData = bme280(BME280_CTRL_HUMIDITY_REG);
	controlData &= 0b11111000; //Clear bits 2/1/0
	controlData |= (uint8_t)humSampling; //Align humSampling to bits 2/1/0
	bme280(BME280_CTRL_HUMIDITY_REG, controlData);

	if(duration > STANDBY_MS_20) duration = STANDBY_MS_0_5; //Error check. Default to 0.5ms
	if(filter > FILTER_X16) filter = FILTER_OFF; //Error check. Default to filter off
	controlData = bme280(BME280_CONFIG_REG);
	controlData &= 0b00000011; //Clear the 7/6/5/4/3/2 bits
	controlData |= ((uint8_t)duration << 5); //Align with bits 7/6/5
	controlData |= ((uint8_t)filter << 2); //Align with bits 4/3/2
	bme280(BME280_CONFIG_REG, controlData);

	// you must make sure to also set REGISTER_CONTROL after setting the
	// CONTROLHUMID register, otherwise the values won't be applied (see
	// DS 5.4.3)
	controlData |= ((uint8_t)tempSampling << 5); //Align tempSampling to bits 7/6/5
	controlData |= ((uint8_t)pressSampling << 2); //Align pressSampling to bits 4/3/2
	if(mode > MODE_NORMAL) mode = MODE_SLEEP; //Error check. Default to sleep mode
	controlData |= (uint8_t)mode; //Set
	bme280(BME280_CTRL_MEAS_REG, controlData);
}

//Set the standby bits in the config register
//tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
void BME280::setStandbyDuration(standby_duration duration)
{
	if(duration > STANDBY_MS_20) duration = STANDBY_MS_0_5; //Error check. Default to 0.5ms
	
	uint8_t controlData = bme280(BME280_CONFIG_REG);
	controlData &= 0b00011111; //Clear the 7/6/5 bits
	controlData |= ((uint8_t)duration << 5); //Align with bits 7/6/5
	bme280(BME280_CONFIG_REG, controlData);
}

//Set the filter bits in the config register
//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8 
//  4, coefficients = 16
void BME280::setFilter(sensor_filter filter)
{
	if(filter > FILTER_X16) filter = FILTER_OFF; //Error check. Default to filter off
	
	uint8_t controlData = bme280(BME280_CONFIG_REG);
	controlData &= 0b11100011; //Clear the 4/3/2 bits
	controlData |= ((uint8_t)filter << 2); //Align with bits 4/3/2
	bme280(BME280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid oversampling values
void BME280::setTempOverSample(sensor_sampling tempSampling)
{	
	sensor_mode originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to tempSampling
	uint8_t controlData = bme280(BME280_CTRL_MEAS_REG);
	controlData &= 0b00011111; //Clear bits 765
	controlData |= ((uint8_t)tempSampling << 5); //Align tempSampling to bits 7/6/5
	bme280(BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid oversampling values
void BME280::setPressureOverSample(sensor_sampling pressSampling)
{	
	sensor_mode originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to pressSampling
	uint8_t controlData = bme280(BME280_CTRL_MEAS_REG);
	controlData &= 0b11100011; //Clear bits 432
	controlData |= ((uint8_t)pressSampling << 2); //Align pressSampling to bits 4/3/2
	bme280(BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid oversampling values
void BME280::setHumidityOverSample(sensor_sampling humSampling)
{
	sensor_mode originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to humSampling
	uint8_t controlData = bme280(BME280_CTRL_HUMIDITY_REG);
	controlData &= 0b11111000; //Clear bits 2/1/0
	controlData |= (uint8_t)humSampling; //Align humSampling to bits 2/1/0
	bme280(BME280_CTRL_HUMIDITY_REG, controlData);

	setMode(originalMode); //Return to the original user's choice
}


//Check the measuring bit and return true while device is taking measurement
bool BME280::isMeasuring(void)
{
	uint8_t stat = bme280(BME280_STAT_REG);
	return(stat & (1<<3)); //If the measuring bit (3) is set, return true
}

//Strictly resets.  Run .begin() afterwards
void BME280::reset(void)
{
	bme280(BME280_RST_REG, 0xB6);
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float BME280::getPressure(void)
{
	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    uint8_t buffer[3];
	bme280(BME280_PRESSURE_MSB_REG, 3, buffer);
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)this->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)this->iCal.dig_P6;
	var2 = var2 + ((var1 * (int64_t)this->iCal.dig_P5)<<17);
	var2 = var2 + (((int64_t)this->iCal.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)this->iCal.dig_P3)>>8) + ((var1 * (int64_t)this->iCal.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)this->iCal.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)this->iCal.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)this->iCal.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)this->iCal.dig_P7)<<4);
	
	return (float)p_acc / 256.0;
}

float BME280::finePressure(void)
{
	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    uint8_t buffer[3];
	bme280(BME280_PRESSURE_MSB_REG, 3, buffer);
    int32_t p_raw = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	// Pressure
	float tf = this->t_fine - 128000.0f;
	float x1 = (tf * this->fcal.p6 + this->fcal.p5) * tf + this->fcal.p4;
	float x2 = (tf * this->fcal.p3 + this->fcal.p2) * tf + this->fcal.p1;

	float pf = ((float) p_raw + x1) / x2;
	float pressure = (pf * this->fcal.p9 + this->fcal.p8) * pf + this->fcal.p7;
	pressure /= 100.0f;	// to mbar

	return pressure;
}

// Sets the internal variable referencePressure so the altitude is calculated properly.
// This is also known as "sea level pressure" and is in Pascals. The value is probably
// within 10% of 101325. This varies based on the weather:
// https://en.wikipedia.org/wiki/Atmospheric_pressure#Mean_sea-level_pressure
//
// if you are concerned about accuracy or precision, make sure to pull the
// "sea level pressure" value from a trusted source like NOAA.
void BME280::setReferencePressure(float refPressure)
{
	this->referencePressure = refPressure;
}

//Return the local reference pressure
float BME280::getReferencePressure()
{
	return(this->referencePressure);
}

float BME280::readAltitudeMeters(void)
{
	float heightOutput = 0;
	
  // Getting height from a pressure reading is called the "international barometric height formula".
  // The magic value of 44330.77 was adjusted in issue #30.
  // There's also some discussion of it here: https://www.sparkfun.com/tutorials/253
  // This calculation is NOT designed to work on non-Earthlike planets such as Mars or Venus;
  // see NRLMSISE-00. That's why it is the "international" formula, not "interplanetary".
  // Sparkfun is not liable for incorrect altitude calculations from this
  // code on those planets. Interplanetary selfies are welcome, however.
	heightOutput = ((float)-44330.77)*(pow(((float)getPressure()/(float)this->referencePressure), 0.190263) - (float)1); //Corrected, see issue 30
	return heightOutput;
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float BME280::getHumidity(void)
{
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    uint8_t buffer[2];
	bme280(BME280_HUMIDITY_MSB_REG, 2, buffer);
    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	
	int32_t var1;
	var1 = (this->t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)this->iCal.dig_H4) << 20) - (((int32_t)this->iCal.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)this->iCal.dig_H6)) >> 10) * (((var1 * ((int32_t)this->iCal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)this->iCal.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)this->iCal.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;
}

float BME280::fineHumidity(void)
{
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    uint8_t buffer[2];
	bme280(BME280_HUMIDITY_MSB_REG, 2, buffer);
	//convert data to number 20 bit
    int32_t h_raw = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	
	float hf = this->t_fine - 76800.0f;
	hf = ((float) h_raw - (this->fcal.h4 + this->fcal.h5 * hf)) * (this->fcal.h2 * (1.0f + this->fcal.h6 * hf * (1.0f + this->fcal.h3 * hf)));
	float humidity = hf * (1.0f - this->fcal.h1 * hf);
	// sanity check
	if (humidity > 100.0f) humidity = 100.0f;
	else if (humidity < 0.0f) humidity = 0.0f;

	return humidity;	// RH = relative humidity in percent
}
		

//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

void BME280::setTemperatureCorrection(float corr)
{
	this->settings.tempCorrection = corr;
}

float BME280::getTempC(void)
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// this->t_fine carries fine temperature as global value

	//get the reading (adc_T);
    uint8_t buffer[3];
	bme280(BME280_TEMPERATURE_MSB_REG, 3, buffer);
    int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)this->iCal.dig_T1<<1))) * ((int32_t)this->iCal.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)this->iCal.dig_T1)) * ((adc_T>>4) - ((int32_t)this->iCal.dig_T1))) >> 12) * ((int32_t)this->iCal.dig_T3)) >> 14;
	this->t_fine = var1 + var2;
	float output = (this->t_fine * 5 + 128) >> 8;

	output = output / 100 + this->settings.tempCorrection;
	
	return output;
}

float BME280::fineTempC(void)
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// this->t_fine carries fine temperature as global value

	//get the reading (t_raw);
    uint8_t buffer[3];
	bme280(BME280_TEMPERATURE_MSB_REG, 3, buffer);
	int32_t t_raw = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	//By datasheet, calibrate
	// Temperature
	float ofs = (float) t_raw - this->fcal.t1;
	this->t_fine = (int32_t)(ofs * this->fcal.t3 + this->fcal.t2) * ofs;
	float temperature = this->t_fine * (1.0f / 5120.0f) + this->settings.tempCorrection;
    if (temperature < temperature_min) {
        temperature = temperature_min;
    } else if (temperature > temperature_max) {
        temperature = temperature_max;
    }
	return temperature;
}

/*!
 *   Calculates the altitude (in meters) from the specified atmospheric
 *   pressure (in hPa), and sea-level pressure (in hPa).
 *   @param  seaLevel      Sea-level pressure in hPa
 *   @returns the altitude value read from the device
 */
float BME280::readAltitude(float seaLevel)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = getPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *   Calculates the pressure at sea level (in hPa) from the specified
 * altitude (in meters), and atmospheric pressure (in hPa).
 *   @param  altitude      Altitude in meters
 *   @param  atmospheric   Atmospheric pressure in hPa
 *   @returns the pressure at sea level (in hPa) from the specified altitude
 */
float BME280::seaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
 *   Returns the current temperature compensation value in degrees Celcius
 *   @returns the current temperature compensation value in degrees Celcius
 */
float BME280::getTemperatureCompensation(void)
{
  return float(((this->t_fine_adjust * 5) >> 8) / 100);
};

/*!
 *  Sets a value to be added to each temperature reading. This adjusted
 *  temperature is used in pressure and humidity readings.
 *  @param  adjustment  Value to be added to each tempature reading in Celcius
 */
void BME280::setTemperatureCompensation(float adjustment)
{
  // convert the value in C into and adjustment to t_fine
  this->t_fine_adjust = ((int32_t(adjustment * 100) << 8)) / 5;
};

float BME280::getVirtTemperature(void)
{
	// Returns speed of sound in centimeters. Output value of “34123” equals 341.23 m/s.
	// this->t_fine carries fine temperature as global value

	//get the reading (t_raw);
    uint8_t buffer[3];
	bme280(BME280_TEMPERATURE_MSB_REG, 3, buffer);
	int32_t t_raw = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	//By datasheet, calibrate
	// Temperature
	float ofs = (float) t_raw - this->fcal.t1;
	this->t_fine = (int32_t)(ofs * this->fcal.t3 + this->fcal.t2) * ofs;
	float temperature = this->t_fine * (1.0f / 5120.0f) + this->settings.tempCorrection;
	// sanity check
    if (temperature < temperature_min) {
        temperature = temperature_min;
    } else if (temperature > temperature_max) {
        temperature = temperature_max;
    }

	bme280(BME280_PRESSURE_MSB_REG, 3, buffer);
    int32_t p_raw = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	// Pressure
	float tf = this->t_fine - 128000.0f;
	float x1 = (tf * this->fcal.p6 + this->fcal.p5) * tf + this->fcal.p4;
	float x2 = (tf * this->fcal.p3 + this->fcal.p2) * tf + this->fcal.p1;

	float pf = ((float) p_raw + x1) / x2;
	float pressure = (pf * this->fcal.p9 + this->fcal.p8) * pf + this->fcal.p7;
	pressure /= 100.0f;	// to mbar

	bme280(BME280_HUMIDITY_MSB_REG, 2, buffer);
	//convert data to number 20 bit
    int32_t h_raw = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	
	float hf = this->t_fine - 76800.0f;
	hf = ((float) h_raw - (this->fcal.h4 + this->fcal.h5 * hf)) * (this->fcal.h2 * (1.0f + this->fcal.h6 * hf * (1.0f + this->fcal.h3 * hf)));
	float humidity = hf * (1.0f - this->fcal.h1 * hf);	// RH = relative humidity in percent
	// sanity check
	if (humidity > 100.0f) humidity = 100.0f;
	else if (humidity < 0.0f) humidity = 0.0f;

	/*
	 * Virtual temperature computation for -35 to +35 degree Celsium with an accuracy of 0.1%
	 * Refer to the Bolton's Formula (1980) for details
	 * https://wahiduddin.net/calc/density_altitude.htm
	 * https://carnotcycle.wordpress.com/2017/08/01/compute-dewpoint-temperature-from-rh-t/
	 * https://journals.ametsoc.org/doi/pdf/10.1175/1520-0493%281980%29108%3C1046%3ATCOEPT%3E2.0.CO%3B2
	 * We can increase an accuracy using Wexler's formula for T >= 0 Celsium
	 */
	const double B  = 6.112; // Bolton's formula multiplier constant
	const double b  = 17.67; // Bolton's numerator multiplier constant
	const double Tb = 243.5; // Bolton's denominator addition constant
	const double p  = 0.378; 
	
	// Bolton's formula for Es = saturation vapor pressure ( multiply mb by 100 to get Pascals)
	float sat_vapor_pressure = (std::exp((b * double(temperature)) / (double(temperature) + Tb))) * B;
	
	// Actual Vapor Pressure from Relative Humidity 
	// Pv = RH * Es where:
	// Pv = pressure of water vapor (partial pressure) in mbar, 
	// RH = relative humidity (expressed as a decimal value)
	// Es = saturation vapor pressure in mbar ( multiply mb by 100 to get Pascals)
	float act_vapor_pressure = (double(humidity) * sat_vapor_pressure) / 100.0F; // in mbar

	float virtTemperature = temperature / (1.0F - (p * act_vapor_pressure) / pressure ); // in Kelvin
	return virtTemperature;
}


// The algorithm is based on the approximate formula published in JASA [1993] by Owen Cramer,
// "The variation of the specific heat ratio and the speed of sound in air with temperature,
// pressure, humidity, and CO2 concentration."
// http://gsd.ime.usp.br/~yili/SpeedOfSound/SpeedOfSound.java
// explained here http://gsd.ime.usp.br/~yili/SpeedOfSound/Speed.html


/**
 * Computes speed of sound
 * @author Yang Yili
 * @param x temperature
 * @param y humidity
 * @return speed speed of sound
 */
float BME280::GetSpeedOfSound(float temp, float humidity)
{
	float speed;
	if(humidity < 0.0) humidity = 0;
	if(humidity > 100.0) humidity = 100.0;
	if((temp > 0.0) && (temp < 30.0)){
		float T = temp + 273.15;
		float h = humidity / 100.0;
		float f = 1.00062 + 0.0000000314 * p + 0.00000056 * temp * temp;
		float Psv = exp(0.000012811805 * T * T - 0.019509874 * T + 
				34.04926034 - 6353.6311 / T);
		
		float Xw = h * f * Psv / p;	//the water vapor mole fraction
		float c = 331.45 - a[0] - p * a[6] - a[13] * p * p;
		c = sqrt(a[9] * a[9] + 4 * a[14] * c);
		
		float Xc = ((-1) * a[9] - c) / ( 2 * a[14]); //the carbon dioxide mole fraction
		speed = a[0] + a[1] * temp + a[2] * temp * temp + 
				(a[3] + a[4] * temp + a[5] * temp * temp) * Xw + 
				(a[6] + a[7] * temp + a[8] * temp * temp) * p + 
				(a[9] + a[10] * temp + a[11] * temp * temp) * Xc + 
				a[12] * Xw * Xw + a[13] * p * p + a[14] * Xc * Xc + 
				a[15] * Xw * p * Xc;

	} else { //below 0*C

	}
	return speed;
} /* method calculateSpeedOfSound */