#ifndef PickupIn_H
#define PickupIn_H

#include "Commands/Command.h"

class PickupIn : public frc::Command {
public:
	PickupIn();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // PickupIn_H
