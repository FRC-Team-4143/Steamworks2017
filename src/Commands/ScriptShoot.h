#ifndef ScriptShoot_H
#define ScriptShoot_H

#include <Commands/CommandGroup.h>

class ScriptShoot : public CommandGroup {
public:
	ScriptShoot(int speed, float timeout);
};

#endif  // ScriptShoot_H
