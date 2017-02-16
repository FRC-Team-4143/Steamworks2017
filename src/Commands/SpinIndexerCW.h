#ifndef SpinIndexerCW_H
#define SpinIndexerCW_H

#include "Commands/Command.h"

class SpinIndexerCW : public frc::Command {
public:
	SpinIndexerCW();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // SpinIndexerCW_H
