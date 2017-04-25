#pragma once
#include "global.h"

class LPF
{
public:
	LPF(float rc)
	 : rc_(rc){
		reset();
	}

	float compute(float value) {
		prev_value_ = prev_value_ * (1 - rc_) + value * rc_;

		return prev_value_;
	}

	float getVal() { return prev_value_; }

	void reset() {
		reset(0);
	}

	void reset(float value) {
		prev_value_ = value;
	}

private:
	float prev_value_;
	float rc_;

	DISALLOW_COPY_AND_ASSIGN(LPF);
};
