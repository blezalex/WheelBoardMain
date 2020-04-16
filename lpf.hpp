#pragma once

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
	BiQuadLpf(const float* frequency_ptr)
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
