#include "Commands/ShootCycle.h"
#include "Commands/Feed.h"
#include "Commands/SetWheelsTwist.h"
#include "Commands/SetWinchPosition.h"
#include "Commands/ShootCyclePart1.h"
#include "Commands/StopShoot.h"

// ==========================================================================

ShootCycle::ShootCycle(int side) {
	AddSequential(new ShootCyclePart1(side));

	AddParallel(new Feed(2));
	AddSequential(new SetWheelsTwist(2));

	AddParallel(new SetWinchPosition(2, false, 0));
	AddSequential(new StopShoot());
}

// ==========================================================================
