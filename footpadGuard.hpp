#pragma once

#include "global.h"
#include "guard.hpp"

class FootpadGuard : public Guard {
public:
	FootpadGuard();

	bool CanStart();
	bool MustStop();

	void Update();

private:
	uint16_t pad_levels_[2];
	uint16_t no_connect_cnt_[2];

	DISALLOW_COPY_AND_ASSIGN(FootpadGuard);
};

