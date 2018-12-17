#ifndef _PID_H
#define _PID_H
#include "global.h"
#include "drv/comms/protocol.pb.h"

class PidController {
public:
	PidController(const Config_PidConfig* params)
		: _params(params) {
	    reset();
	}

	float compute(float error) {
		return compute(error, error - _prevError);
	}

	float compute(float error, float de) {
		float out = error * _params->p + de * _params->d + _sumI * _params->i;

		_prevError = error;
		_sumI = constrain(_sumI + error, -_params->max_i, _params->max_i);

		return out;
	}

	void reset() {
		_sumI = 0;
		_prevError = 0;
	}

private:
	const Config_PidConfig* _params;
	float _prevError;
	float _sumI;

	DISALLOW_COPY_AND_ASSIGN(PidController);
};

#endif
