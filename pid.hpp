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

	float compute(float error, float de, float ierror) {
		ierror =  constrain(ierror, -_params->max_i, _params->max_i);
		_sumI = constrain(_sumI +  ierror * _params->i, -MOTOR_CMD_RANGE, MOTOR_CMD_RANGE);

		return  error * _params->p + de * _params->d + _sumI;;
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
