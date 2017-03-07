#pragma once

#include <Commands/CommandGroup.h>

class ScriptShoot : public CommandGroup {
public:
	ScriptShoot(int speed, float timeout);
};
