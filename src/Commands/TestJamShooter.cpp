#include "Commands/TestJamShooter.h"
#include "Robot.h"

// ==========================================================================

TestJamShooter::TestJamShooter() {
}

// ==========================================================================

void TestJamShooter::Initialize() {
	Robot::indexer->testJamShooter();
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
	Robot::indexer->testJamShooter();
}

// ==========================================================================

void TestJamShooter::Interrupted() {
	End();
}

// ==========================================================================
