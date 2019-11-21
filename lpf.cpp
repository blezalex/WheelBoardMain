#include "lpf.hpp"
#include <math.h>

#define PI 3.1415926

// based on http://www.ti.com/lit/an/slaa447/slaa447.pdf
void biquad_butterworth_init(float cutoffFreqHz, float samplingFreqHz, float* filter_params) {
	float Q = 0.707;

	float W0 = 2 * PI * cutoffFreqHz / samplingFreqHz;
	float cosW0 = cos(W0);
	float alpha = sin(W0) / (2 * Q);

	float b0 = (1 - cosW0) / 2;
	float b1 = (1 - cosW0);
	float b2 = (1 - cosW0) / 2;

	float a0 = 1 + alpha;
	float a1 = -2 * cosW0;
	float a2 = 1 - alpha;

	// normalize by a0
	b0 /= a0;
	b1 /= a0;
	b2 /= a0;

	a1 /= -a0;
	a2 /= -a0;

	filter_params[0] = b0;
	filter_params[1] = b1;
	filter_params[2] = b2;

	filter_params[3] = a1;
	filter_params[4] = a2;
}

// state xi-1, xi-2, yi-1, yi-2
float biquad_compute(float in, float* params, float* state) {
	float result = params[0] * in + params[1] * state[0] + params[2] * state[1] + params[3] * state[2] + params[4] * state[3];

	// Propagate X state
	state[1] = state[0];
	state[0] = in;

	// Propagate Y state
	state[3] = state[2];
	state[2] = result;

	return result;
}
