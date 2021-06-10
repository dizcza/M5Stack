#ifndef RCWL9600_h
#define RCWL9600_h

#include <Arduino.h>
#include <Wire.h>


const uint8_t adrRCWL9600 = 0x57;
	
class RCWL9600 {

	public:
		RCWL9600();
		
		void begin(uint8_t halfWayDistance = 170);
		bool sendPing(void);
		uint32_t getDistance(void);

	private:
		uint8_t _distance;

};

#endif //RCWL9600_h
