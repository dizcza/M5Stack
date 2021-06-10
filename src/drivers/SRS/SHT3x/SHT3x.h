/*
	Arduino library for Sensirion temperature and humidity sensors SHT30, SHT31 & SHT35.
	the heavy version.
	Check for /examples for examples of different use cases.
	
	The datasheet I followed is:
	https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Sensirion_Humidity_Sensors_SHT31_Datasheet_digital.pdf
	For more simple version check the SimpleSHT31 library.
	
	The constructor structure:
	SHT3x(	int Address = 0x44, //I2C device address, 0x44 or 0x45
				ValueIfError Value = Zero, //What to return in case of errors. Zero or PrevValue
				SHT31Sensor SensorType = SHT30, //Sensor type, SHT30, SHT31 or SHT35.
				SHT3xMode Mode=Single_HighRep_ClockStretch //Operation mode , look for "enum SHT3xMode"
				); 
	
	Supports:
		Temperature data at Celsius scales.
		Relative humidity data.
		Data integrity (by CRC8 algorithm) (datasheet/section 4.12).
		Calibration (linear) of temperature and humidity data by factors or by reverse sensor values (2 points)
		Heater On/Off (integrated to SHT31 sensor) (datasheet/section 4.10)
		Different sensor actions modes (datasheet/section 4.3)
		Soft reset
		
	Do not supports:
		Action in periodic mode (datasheet/section 4.5)
		Interrupts (datasheet/section 3.5)

	Note 1: by default, the data from sensor updates not faster, than 2 times a second.
	For faster update use setUpdateInterval(uint32_t UpdateIntervalMillisec); but do not exceed the datasheet values (10 measurments per second (100 ms)) because of sensor self-heating (datasheet/section 4.5, at the end of Table 9)
	
	Note 2: The sensor type affects the tolerance values only. 
	
	
	Created by Risele for everyone's use (profit and non-profit).

	ALL THESE WOR_DS
	ARE YOURS EXCEPT
	RISELE
	ATTEMPT NO
	namechangING THERE
	USE THEM TOGETHER
	USE THEM IN PEACE
	
*/

#ifndef SHT3x_h
#define SHT3x_h

#include <Arduino.h>
#include <Wire.h>


const uint8_t adrSht0x44 = 0x44;
const uint8_t adrSht0x45 = 0x45;
	
class SHT3x {

	public:
		enum SHT3xMode
		{
			Single_HighRep_ClockStretch,
			Single_MediumRep_ClockStretch,
			Single_LowRep_ClockStretch,
			Single_HighRep_NoClockStretch,
			Single_MediumRep_NoClockStretch,
			Single_LowRep_NoClockStretch	
		};

		struct CalibrationPoints
		{
			float First;
			float Second;
		};
		
		struct CalibrationFactors
		{
			CalibrationFactors():Factor(1.), Shift(0.){}
			float Factor;
			float Shift;
		};


		SHT3x(const uint8_t addr = adrSht0x44);
		
		void begin(SHT3xMode mode = Single_HighRep_ClockStretch);
		void setMode(SHT3xMode mode = Single_HighRep_ClockStretch);

		void reset(void);
		void heaterOn(void);
		void heaterOff(void);
		
		void setUpdateInterval(uint32_t UpdateIntervalMillisec);
		void setTimeout(uint32_t TimeoutMillisec);

		void setTemperatureCalibrationFactors(CalibrationFactors TemperatureCalibration);
		void setRelHumidityCalibrationFactors(CalibrationFactors RelHumidityCalibration);
		void setTemperatureCalibrationPoints(CalibrationPoints SensorValues, CalibrationPoints Reference);
		void setRelHumidityCalibrationPoints(CalibrationPoints SensorValues, CalibrationPoints Reference);

		bool  updateData(void);
		float getTempC(void);
		float getHumidity(void);
		float getSpeedOfSound(float temperature, float humidity);
	private:
		// The sensor model number
		uint8_t chipID;
		// The address of this device
		uint8_t addr;
		// The register to write to
		uint8_t reg;
		uint8_t value;
        // Internal buffer to reuse for reads
        uint8_t data[6];

		CalibrationFactors temperatureCalibration;
		CalibrationFactors humidityCalibration;
		
		float temperature;
		float humidity;

		uint32_t updateIntervalMilli = 500;
		uint32_t timeoutMillis = 100;

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

		uint8_t sht3x(uint8_t reg);
		bool    sht3x(uint8_t reg, uint8_t value);

};

#endif //SHT3x_h
