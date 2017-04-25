#pragma once
#include <stdint.h>
#include "global.h"

class PwmOut {
public:
	PwmOut() {}

	void init(uint16_t val);

	void set(uint16_t val);
	uint16_t get();

private:
	DISALLOW_COPY_AND_ASSIGN(PwmOut);
};
