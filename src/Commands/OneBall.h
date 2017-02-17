#ifndef OneBall_H
#define OneBall_H

#include "Commands/Command.h"

class OneBall : public frc::Command {
public:
	OneBall();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // OneBall_H
