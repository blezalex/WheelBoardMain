#include "ledController.hpp"



#include "drv/led/led.hpp"
#include "arduino.h"
#include <math.h>

#define STOPPED_SPEED  100 // speed at which board is considered stopped
#define TURN_TILT_ANGLE 3

enum DriveState {
	Stopped, // Display battery level?
	Fwd,
	Reverse
};

enum TurnState {
	None,
	Left,
	Rigth
};

struct LedState {
	DriveState drive_state;
	TurnState turn_state;
	bool braking = false;
	float prev_speed;
};

LedState led_state;


void led_controller_set_state(float speed, float tilt) {
	led_state.braking = (fabs(speed)  < fabs(led_state.prev_speed) * 0.95); // if speed reduced by 5% or more

	if (abs(speed) < STOPPED_SPEED) {
		led_state.drive_state = DriveState::Stopped;
	}
	else {
		led_state.drive_state = speed > 0 ? DriveState::Fwd : DriveState::Reverse;
	}

	led_state.prev_speed = speed;

	if (tilt > TURN_TILT_ANGLE) {
		led_state.turn_state = TurnState::Left;
	}
	else if (tilt < -TURN_TILT_ANGLE) {
		led_state.turn_state = TurnState::Rigth;
	} else {
		led_state.turn_state = TurnState::None;
	}
}

#define LED_COUNT_SINGLE_SIDE (LED_COUNT/2)

#define COLOR_YELLOW 0x909F00

void led_controller_update() {
	static uint16_t prev_time = 0;

	uint16_t time = millis();
	if (time - prev_time < 50u) {
		return;
	}

	prev_time = time;

	if (led_state.drive_state == DriveState::Stopped) {
		for (int i = 0; i < LED_COUNT; i++) {
			led_set_color(i, 0xFFFFFF);
		}
		return;
	}

	static int led_idx = 0;
	if (led_state.turn_state != TurnState::None ) {
		if (led_idx >= LED_COUNT_SINGLE_SIDE) {
			for (int i = 0; i < LED_COUNT; i++) {
				led_set_color(i, 0);
			}
			led_idx = 0;
		}
		else {
			if (led_state.turn_state == TurnState::Left) {
				led_set_color(led_idx, COLOR_YELLOW);
				led_set_color(LED_COUNT_SINGLE_SIDE + led_idx, COLOR_YELLOW);
			}
			else {
				led_set_color(LED_COUNT_SINGLE_SIDE - led_idx - 1, COLOR_YELLOW);
				led_set_color(LED_COUNT_SINGLE_SIDE + LED_COUNT_SINGLE_SIDE - led_idx - 1, COLOR_YELLOW);
			}

			led_idx++;
		}

		return;
	}

	led_idx = LED_COUNT_SINGLE_SIDE;
	int32_t front_color = led_state.drive_state == DriveState::Fwd ? 0xFFFFFF : 0x00FF00;
	int32_t back_color = led_state.drive_state == DriveState::Reverse ? 0xFFFFFF : 0x00FF00;
	for (int i = 0; i < LED_COUNT_SINGLE_SIDE; i++) {
		led_set_color(i, front_color);
	}
	for (int i = 0; i < LED_COUNT_SINGLE_SIDE; i++) {
		led_set_color(LED_COUNT_SINGLE_SIDE + i, back_color);
	}
}

void led_controller_init() {
	led_init();
}
