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

	float compute(float value, bool asym)
	{
		if (value > _prevValue)
		{
			float rcUp = *_rc * 2;
			if (rcUp > 0)
				rcUp = 1;

			_prevValue = _prevValue * (1-rcUp) + value * rcUp;
		}
		else {
			_prevValue = _prevValue * (1-*_rc) + value * *_rc;
		}

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
