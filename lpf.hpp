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
