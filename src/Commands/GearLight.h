#ifndef GearLight_H
#define GearLight_H

#include "Commands/Command.h"

class GearLight : public frc::Command {
public:
	GearLight();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // GearLight_H
