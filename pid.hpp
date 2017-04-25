#ifndef _PID_H
#define _PID_H
#include "global.h"

struct PidSettings
{
  PidSettings() {
  	PidSettings(0, 0, 0, 0);
  }

  PidSettings(float p, float d, float i, float maxI) {
    P = p;
    D = d;
    I = i;
    MaxI = maxI;
  }

  float P;
  float D;
  float I;
  float MaxI;
};

class PidController {
public:
	PidController(const PidSettings& params)
		: _params(params) {
	    reset();
	}

	float compute(float error) {
		return compute(error, error - _prevError);
	}

	float compute(float error, float de) {
		float out = error * _params.P + de * _params.D + _sumI * _params.I;

		_prevError = error;
		_sumI = constrain(_sumI + error, -_params.MaxI, _params.MaxI);

		return out;
	}

	void reset() {
		_sumI = 0;
		_prevError = 0;
	}

private:
	const PidSettings& _params;
	float _prevError;
	float _sumI;

	DISALLOW_COPY_AND_ASSIGN(PidController);
};

#endif
