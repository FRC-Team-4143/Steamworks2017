#ifndef FuelLight_H
#define FuelLight_H

#include "Commands/Command.h"

class FuelLight : public frc::Command {
public:
	FuelLight();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // FuelLight_H
