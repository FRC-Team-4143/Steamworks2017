#ifndef Climb_H
#define Climb_H

#include "Commands/Command.h"

class Climb : public frc::Command {
public:
	Climb();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Climb_H
