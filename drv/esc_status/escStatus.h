#pragma once

struct ESCMessage {
	int8_t speed;				// speed in hal changes per 50ms
	uint8_t batteryVoltageX425; // battery voltage in volts multiplied by 4.25
	uint8_t escTemp;			// temp in Celsius
	uint8_t checkSum;			// 31 xor all other fields
};
