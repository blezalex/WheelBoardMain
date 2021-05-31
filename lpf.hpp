#pragma once

#include <math.h>
#include <arduino.h>

class LPF
{
public:
	LPF(const float* rc)
	{
		_rc = rc;
		reset();
	}

	float compute(float value)
	{
		_prevValue = _prevValue * (1-*_rc) + value * *_rc;
		return _prevValue;
	}

	float getVal() const { return _prevValue; }

	void reset() {
		reset(0);
	}

	void reset(float value) {
		_prevValue = value;
	}

	const float *_rc;

private:
	float _prevValue;
};


void biquad_butterworth_init(float cutoffFreqHz, float samplingFreqHz, float* filter_params);
float biquad_compute(float in, float params[5], float state[4]);



class BiQuadLpf {
public:
	explicit BiQuadLpf(const float* frequency_ptr)
		: frequency_ptr_(frequency_ptr) { }

	void reset() {
		reset(0);
	}

	void reset(float value) {
		for (int i = 0; i < 4; i++) {
			bw_state_[i] = value;
		}
	}

	float compute(float value)
	{
		const bool params_changed = prev_rc_ != *frequency_ptr_;
		if (params_changed) {
			prev_rc_ = *frequency_ptr_;
			biquad_butterworth_init(prev_rc_, 1000, bw_params_);
		}

		return prev_value_ = biquad_compute(value, bw_params_, bw_state_);
	}

	float getVal() const { return prev_value_; }


private:
	float prev_rc_ = -1;
	const float* frequency_ptr_;
	float bw_params_[5];
	float bw_state_[4];

	float prev_value_;
};


static const float kLoopRateMult = 0.001;

class Ramp {
public:


	explicit Ramp(const float* raise_rate, const float* drop_rate, float* limit) : raise_rate_(raise_rate), drop_rate_(drop_rate), limit_(limit) {

	}

	void Reset() {
		value_ = 0;
	}

	float Compute(float new_value) {
		const bool new_positive = new_value > 0;
		const bool old_positive  = value_ > 0;
	  float max_rate;

	  if (new_value == 0) {
	  	max_rate = *drop_rate_;
	  }
	  else {
	  	max_rate = new_positive != old_positive || fabsf(new_value) > fabsf(value_) ? *raise_rate_  : *drop_rate_;
	  }

	  max_rate *= kLoopRateMult;

	  value_ += constrain((new_value - value_), -max_rate, max_rate );
	  value_ = constrain(value_, -*limit_, *limit_);
		return value_;
	}

private:
	const float* raise_rate_;
	const float* drop_rate_;
	const float* limit_;

	float value_;
};
