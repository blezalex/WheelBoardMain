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
	BiQuadLpf(const float* frequency_ptr) :legacy_(frequency_ptr) { }

	void reset() {
		reset(0);
	}

	void reset(float value) {
		legacy_.reset(value);

		for (int i = 0; i < 4; i++) {
			bw_state_[i] = value;
		}
	}

	float compute(float value)
	{
		const bool use_biguad = *legacy_._rc > 1;
		if (!use_biguad) {
			return prev_value_ = legacy_.compute(value);
		}

		const bool params_changed = prev_rc_ != *legacy_._rc;
		if (params_changed) {
			biquad_butterworth_init(*legacy_._rc, 1000, bw_params_);
			prev_rc_ = *legacy_._rc;
		}

		return prev_value_ = biquad_compute(value, bw_params_, bw_state_);
	}

	float getVal() const { return prev_value_; }


private:
	float prev_rc_ = -1;
	float bw_params_[5];
	float bw_state_[4];

	LPF legacy_;
	float prev_value_;
};
