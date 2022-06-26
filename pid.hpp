#ifndef _PID_H
#define _PID_H
#include "global.h"
#include "drv/comms/config.pb.h"

class PidController {
public:
	PidController(const Config_PidConfig* params)
		: _params(params) {
	    reset();
	}

	float compute(float error) {
		float de = error - _prev_error;
		_prev_error = error;

		return compute(error, de);
	}

	float compute(float error, float de) {
		// Cumulative sumI changes slowly, it is OK to use I value from previous iteration here.
		float output =  error * _params->p + de * _params->d + applyExpoPoly(_sumI,_params->i_expo);

		float ierror = constrain(error, -_params->max_i, _params->max_i);
		ierror = error * _params->i;
		// Accumulate I unless windup is detected.
		if ((output < 1 && output > -1) ||
				(output > 1 && ierror < 0) ||
				(output < -1 && ierror > 0)) {
			_sumI += ierror;
			_sumI = constrain(_sumI, -1, 1); // limit to range
		}

		return output;
	}

	void reset() {
		resetI();
		_prev_error = 0;
	}

	void resetI() {
		_sumI = 0;
	}

private:
	const Config_PidConfig* _params;
	float _sumI;
	float _prev_error;

	DISALLOW_COPY_AND_ASSIGN(PidController);
};

#endif
