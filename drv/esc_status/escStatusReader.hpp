#pragma once
#include "global.h"
#include "escStatus.h"

class EscStatusReader {
public:
	EscStatusReader(Usart* escChannel) : escChannel_(escChannel) { }

	bool update() {
		return false;
	}


private:
	Usart* escChannel_;
};
