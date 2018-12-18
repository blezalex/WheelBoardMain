#pragma once

#include "global.h"
#include "guard.hpp"
#include "lpf.hpp"
#include "drv/comms/protocol.pb.h"

class FootpadGuard : public Guard {
public:
	FootpadGuard(const Config_FootPadSettings* settings);

	bool CanStart();
	bool MustStop();

	void Update();

	uint16_t getLevel(int i) {
		return (uint16_t)padLevelFilter[i].getVal();
	}

private:
	LPF padLevelFilter[2];

	const Config_FootPadSettings* settings_;

	DISALLOW_COPY_AND_ASSIGN(FootpadGuard);
};

