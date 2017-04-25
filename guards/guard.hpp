#pragma once
#include "global.h"

class Guard {
public:
	virtual void Update() { };
	virtual bool CanStart() = 0;
	virtual bool MustStop() = 0;
};
