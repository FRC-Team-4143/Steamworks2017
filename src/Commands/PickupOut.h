#ifndef PickupOut_H
#define PickupOut_H

#include "Commands/Command.h"

class PickupOut : public frc::Command {
public:
	PickupOut();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // PickupOut_H
