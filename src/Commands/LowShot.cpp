#include "Commands/LowShot.h"
#include "Commands/Feed.h"
#include "Commands/LowGoalShoot.h"
#include "Commands/PositionDrive.h"
#include "Commands/SetWheelsTwist.h"
#include "Commands/SetWinchPosition.h"


// ==========================================================================

LowShot::LowShot() {
	// Add Commands here:
	// e.g. AddSequential(new Command1());
	//      AddSequential(new Command2());
	// these will run in order.

	// To run multiple commands at the same time,
	// use AddParallel()
	// e.g. AddParallel(new Command1());
	//      AddSequential(new Command2());
	// Command1 and Command2 will run in parallel.

	// A command group will require all of the subsystems that each member
	// would require.
	// e.g. if Command1 requires chassis, and Command2 requires arm,
	// a CommandGroup containing them would require both the chassis and the
	// arm.

	AddParallel(new PositionDrive(40));
	AddSequential(new SetWinchPosition(1, true, 2));

	AddParallel(new LowGoalShoot(2));
	AddParallel(new Feed(2));
	AddSequential(new SetWheelsTwist(2));

	AddParallel(new SetWinchPosition(2, false, 4));
	//AddSequential(new StopShoot());
}

// ==========================================================================
