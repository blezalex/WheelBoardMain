#include "ledController.hpp"



#include "drv/led/led.hpp"
#include "arduino.h"
#include "lpf.hpp"
#include <math.h>

#define STOPPED_SPEED  300 // speed at which board is considered stopped
#define TURN_INDICATION_MIN_RATE_DEG_SEC 45

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
	float battery_level;
};

LedState led_state;

float rotation_rc = 0.1;
LPF rotation_lpf(&rotation_rc);


void led_controller_set_state(float speed, float rotation_rate, float battery_level) {
	led_state.braking = (fabsf(speed)  < fabsf(led_state.prev_speed) * 0.95); // if speed reduced by 5% or more
	led_state.battery_level = battery_level;

	if (abs(speed) < STOPPED_SPEED) {
		led_state.drive_state = DriveState::Stopped;
	}
	else {
		led_state.drive_state = speed < 0 ? DriveState::Fwd : DriveState::Reverse;
	}

	led_state.prev_speed = speed;

	rotation_rate = rotation_lpf.compute(rotation_rate);
	if (rotation_rate > TURN_INDICATION_MIN_RATE_DEG_SEC) {
		led_state.turn_state = TurnState::Left;
	}
	else if (rotation_rate < -TURN_INDICATION_MIN_RATE_DEG_SEC) {
		led_state.turn_state = TurnState::Rigth;
	} else {
		led_state.turn_state = TurnState::None;
	}
}

#define LED_COUNT_SINGLE_SIDE (LED_COUNT/2)

#define COLOR_YELLOW 0x909F00

void led_controller_update() {
	static uint16_t prev_time = 0;

	uint16_t time = half_millis();
	if (time - prev_time < 80u) {
		return;
	}

	prev_time = time;

	static int led_idx = 0;
	if (led_state.turn_state != TurnState::None ) {
		if (led_idx >= LED_COUNT_SINGLE_SIDE) {
			for (int i = 0; i < LED_COUNT; i++) {
				led_set_color(i, 0);
			}
			led_idx = 0;
		}
		else {
			if ((led_state.turn_state == TurnState::Left && led_state.drive_state == DriveState::Reverse)
					||
					(led_state.turn_state == TurnState::Rigth && led_state.drive_state == DriveState::Fwd)) {
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

	if (led_state.drive_state == DriveState::Stopped) {

		int green_led_cnt = led_state.battery_level * LED_COUNT_SINGLE_SIDE;

		for (int i = 0; i < LED_COUNT_SINGLE_SIDE; i++) {
			uint32_t color = i < green_led_cnt ? 0xDF0000 : 0x00DF00;
			led_set_color(i, color);
			led_set_color(LED_COUNT - i - 1, color);
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

void led_controller_startup_animation() {
	static uint16_t prev_time = 0;
	uint16_t time = half_millis();
	if (time - prev_time < 150u) {
		return;
	}
	prev_time = time;

	for (int i = 0; i < LED_COUNT; i++) {
		led_set_color(i, 0);
	}

	for (int i = 0; i < 4; i++) {
		int led = rand() % LED_COUNT;
		int clr = rand() % (1 << 4);
		led_set_color(led,
					((clr & (1)) ? 0x30 : 0)
				| ((clr & (1 << 1)) ? 0x3000 : 0)
				| ((clr & (1 << 2)) ? 0x300000 : 0) );
	}
}

void led_controller_init() {
	led_init();
  rotation_rc = 0.12;

  rotation_lpf = LPF(&rotation_rc);
	rotation_lpf.reset();
}
