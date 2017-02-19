#ifndef ReverseClimb_H
#define ReverseClimb_H

#include "Commands/Command.h"

class ReverseClimb : public frc::Command {
public:
	ReverseClimb();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Climb_H
