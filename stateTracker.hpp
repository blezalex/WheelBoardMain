#pragma once
#include "global.h"
#include "guards/guard.hpp"
#include "arduino.h"

enum class State {
	Stopped,
	FirstIteration,
	Starting,
	Running
};

class StateTracker {
public:
	StateTracker(Guard** guards, int guards_count)
		: state_(State::Stopped), guards_(guards), guards_count_(guards_count) { }

	State update() {
		for (int i = 0; i < guards_count_; i++) {
			Guard* g = guards_[i];
			g->Update();
		}

		switch (state_) {
		case State::Starting: handleStartingState(); break;
		case State::FirstIteration: handleFirstInterationState(); break;
		case State::Stopped: handleStoppedState(); break;
		}

		if (state_ != State::Stopped) {
			for (int i = 0; i < guards_count_; i++) {
				if (guards_[i]->MustStop()) {
					state_ = State::Stopped;
					break;
				}
			}
		}

		return state_;
	}

	float start_progress(){
		return start_progress_;
	}

private:
	void handleStartingState() {
		uint16_t durationSinceStart = millis() - start_timestamp_;
		if (durationSinceStart > START_DURATION) {
			state_ = State::Running;
			start_progress_ = 1;
		}
		start_progress_ = durationSinceStart / (float)START_DURATION;
	}

	void handleFirstInterationState() {
		state_ = State::Starting;
	}

	void handleStoppedState() {
		for (int i = 0; i < guards_count_; i++) {
			if (!guards_[i]->CanStart()) {
				return;
			}
		}

		state_ = State::FirstIteration;
		start_timestamp_ = millis();
		start_progress_ = 0;
	}


private:
	State state_;
	uint16_t start_timestamp_;
	float start_progress_; // [0:1] interval
	Guard** guards_;
	int guards_count_;

	DISALLOW_COPY_AND_ASSIGN(StateTracker);
};
