#include "SHT3x.h"
#include "CRC8.h"

SHT3x::SHT3x(const uint8_t addr) {
	this->addr = addr;
}

void SHT3x::begin(SHT3xMode mode)
{
	setMode(mode);
}


/*  Read data back from the device

    @returns true iff all words read and CRC passed
*/
bool SHT3x::updateData()
{
    size_t read;
    uint8_t crc = 0xFF;
    uint8_t *next;
    bool success = true;

    // Clear buffer
    for (next = &this->data[5]; next > this->data; next--) *next = 0;
	//write preset mode to the device
	sht3x(this->reg, this->value);
    // Each word is two bytes plus a CRC byte, ergo 3 bytes per word
    read = Wire.requestFrom((uint8_t)this->addr, uint8_t(6));

    /*  We should have read the requested number of bytes.
        If not, we need to clear the bytes read anyways.
    */
    if (read != 6) success = false;;

    /*  Calculate CRC while reading bytes
        Adapted from:
        http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
    */
    next = this->data;
    for (; read > 0; read--) {
        // Read next available byte
        *next = Wire.read();
        // Every third byte
        if ((read % 3) == 1) {
            // Check CRC byte
            success = success && (crc == *next);
            crc = 0xFF;
        } else {
            // Update CRC
            crc = CRC_LUT[crc ^ *next];
            // Go to next byte
            next++;
        }
    }
	if (success) {
		uint16_t TemperatureRaw = (data[0]<<8) + (data[1]<<0);
		uint16_t RelHumidityRaw = (data[3]<<8) + (data[4]<<0);
		this->temperature =	((float) TemperatureRaw) * 0.00267033 - 45.;
		this->temperature =	this->temperature * this->temperatureCalibration.Factor +
							this->temperatureCalibration.Shift;
		this->humidity =	((float) RelHumidityRaw) * 0.0015259;
		this->humidity =	this->humidity * this->humidityCalibration.Factor +
							this->humidityCalibration.Shift;
	} else {
		Serial.println("SHT CRC error!");
		this->temperature = 0;
		this->humidity =	0;
	}
    return success;
}

float SHT3x::getTempC()
{
	return this->temperature;
}

float SHT3x::getHumidity()
{
	return this->humidity;
}

float SHT3x::getSpeedOfSound(float temp, float humidity)
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

void SHT3x::setMode(SHT3xMode mode)
{
	switch (mode) {
		case Single_HighRep_ClockStretch:
		{
			this->reg=0x2C;
			this->value=0x06;
			break;
		}
		case Single_MediumRep_ClockStretch:
		{
			this->reg=0x2C;
			this->value=0x0D;
			break;
		}
		case Single_LowRep_ClockStretch:
		{
			this->reg=0x2C;
			this->value=0x10;
			break;
		}
		case Single_HighRep_NoClockStretch:
		{
			this->reg=0x24;
			this->value=0x00;
			break;
		}
		case Single_MediumRep_NoClockStretch:
		{
			this->reg=0x24;
			this->value=0x0B;
			break;
		}
		case Single_LowRep_NoClockStretch:
		{
			this->reg=0x24;
			this->value=0x16;
			break;
		}
		default:
		{
			this->reg=0x2C;
			this->value=0x06;
			break;
		}
	}
}

void SHT3x::setTemperatureCalibrationFactors(CalibrationFactors TemperatureCalibration)
{
	this->temperatureCalibration = TemperatureCalibration;
}

void SHT3x::setRelHumidityCalibrationFactors(CalibrationFactors RelHumidityCalibration)
{
	this->humidityCalibration = RelHumidityCalibration;
}

void SHT3x::setTemperatureCalibrationPoints(CalibrationPoints SensorValues, CalibrationPoints Reference)
{
	this->temperatureCalibration.Factor = (Reference.Second - Reference.First) / (SensorValues.Second - SensorValues.First);
	this->temperatureCalibration.Shift = Reference.First - this->temperatureCalibration.Factor * SensorValues.First;
}

void SHT3x::setRelHumidityCalibrationPoints(CalibrationPoints SensorValues, CalibrationPoints Reference)
{
	this->humidityCalibration.Factor = (Reference.Second - Reference.First) / (SensorValues.Second - SensorValues.First);
	this->humidityCalibration.Shift = Reference.First - this->humidityCalibration.Factor * SensorValues.First;
}

void SHT3x::reset()
{
	sht3x(0x30, 0xA2);
}

void SHT3x::heaterOn()
{
	sht3x(0x30, 0x6D);
}

void SHT3x::heaterOff()
{
	sht3x(0x30, 0x66);
}


void SHT3x::setUpdateInterval(uint32_t UpdateIntervalMillisec)
{
	if (UpdateIntervalMillisec > 0) {
		this->updateIntervalMilli = UpdateIntervalMillisec;
	}
}

void SHT3x::setTimeout(uint32_t TimeoutMillisec)
{
	if (TimeoutMillisec > 0) {
		this->timeoutMillis = TimeoutMillisec;
	}
}

// Single register read and write
uint8_t SHT3x::sht3x(uint8_t reg)
{
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(this->addr, uint8_t(1));
	return Wire.read();
}

bool SHT3x::sht3x(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(this->addr);
	Wire.write(reg);
	Wire.write(value);
	return (Wire.endTransmission() == 0);
}

