#include "Commands/TestJamShooter.h"
#include "Robot.h"

// ==========================================================================

TestJamShooter::TestJamShooter() {
}

// ==========================================================================

void TestJamShooter::Initialize() {
	Robot::indexer->TestJamShooter();
}

// ==========================================================================

void TestJamShooter::Execute() {
}

// ==========================================================================

bool TestJamShooter::IsFinished() {
	return true;
}

// ==========================================================================

void TestJamShooter::End() {
	Robot::indexer->TestJamShooter();
}

// ==========================================================================

void TestJamShooter::Interrupted() {
	End();
}

// ==========================================================================
