#ifndef _PID_H
#define _PID_H

#include "drv/comms/protocol.pb.h"

class PidController {
public:
	PidController(const Config_PidConfig* params)
		: _params(params) {
	    reset();
	}

	float compute(float error, float de) {
		float ierror = constrain(error, -_params->max_i, _params->max_i);

		ierror = ierror * _params->i / 100; // scale it up by I, and by 100 to get close to old configs

		_sumI = constrain(_sumI, -1, 1); // limit to range

		// Cumulative sumI changes slowly, it is OK to use I value from previous iteration here.
		float output =  error * _params->p + de * _params->d + applyExpoPoly(_sumI,_params->i_expo);

		// Accumulate I unless windup is detected.
		if ((output < 1 && output > -1) ||
				(output > 1 && ierror < 0) ||
				(output < -1 && ierror > 0)) {
			_sumI += ierror;
		}

		return output;
	}

	void reset() {
		_sumI = 0;
	}

private:
	const Config_PidConfig* _params;
	float _sumI;

	DISALLOW_COPY_AND_ASSIGN(PidController);
};

#endif
