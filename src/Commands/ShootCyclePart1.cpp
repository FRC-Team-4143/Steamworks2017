#include "Commands/ShootCyclePart1.h"
#include "Commands/DefeedShoot.h"
#include "Commands/PositionDrive.h"
#include "Commands/SetWinchPosition.h"

// ==========================================================================

ShootCyclePart1::ShootCyclePart1(int side) {
	AddParallel(new SetWinchPosition(3, true, 1));
	AddParallel(new DefeedShoot(2));
	AddSequential(new PositionDrive(0, side));
}

// ==========================================================================
