#pragma once

#include <Commands/CommandGroup.h>

class ScriptShootWithVision : public CommandGroup {
public:
	ScriptShootWithVision(int speed, float timeout);
};
