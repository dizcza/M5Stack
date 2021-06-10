#include "RCWL9600.h"


RCWL9600::RCWL9600() {
}

void RCWL9600::begin(uint8_t halfWayDisctance)
{
	_distance = halfWayDisctance << 1;
}

uint32_t RCWL9600::getDistance(void) {
	uint32_t data;
    Wire.requestFrom(adrRCWL9600,uint8_t(3));
    data  = Wire.read(); data <<= 8;
    data |= Wire.read(); data <<= 8;
    data |= Wire.read();
    return data;
}

bool RCWL9600::sendPing(void) {
	Wire.beginTransmission(adrRCWL9600);
	Wire.write(0x01);
	return (Wire.endTransmission() == 0);
}

