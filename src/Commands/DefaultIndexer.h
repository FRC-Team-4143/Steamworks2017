#ifndef DefaultIndexer_H
#define DefaultIndexer_H

#include "Commands/Command.h"

class DefaultIndexer : public frc::Command {
public:
	DefaultIndexer();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DefaultIndexer_H
