#include "ZeroReverse.h"
#include "Subsystems/Indexer.h"
#include "Robot.h"

ZeroReverse::ZeroReverse() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

void ZeroReverse::Initialize() {
	Robot::indexer->ZeroTimer();
}

void ZeroReverse::Execute() {
}

bool ZeroReverse::IsFinished() {
	return true;
}

void ZeroReverse::End() {
}

void ZeroReverse::Interrupted() {
	End();
}
