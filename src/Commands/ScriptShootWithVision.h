#ifndef ScriptShoot_H
#define ScriptShoot_H

#include <Commands/CommandGroup.h>

class ScriptShootWithVision : public CommandGroup {
public:
	ScriptShootWithVision(int speed, float timeout);
};

#endif  // ScriptShoot_H
