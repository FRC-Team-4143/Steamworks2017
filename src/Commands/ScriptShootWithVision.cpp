#include "BoilerLineup.h"
#include "ScriptShootWithVision.h"
#include "ScriptStartShooter.h"
#include "ScriptStopShooter.h"
#include "ScriptIndex.h"

ScriptShootWithVision::ScriptShootWithVision(int speed, float timeout) {
	AddSequential(new BoilerLineup(0, 0));
	AddSequential(new ScriptStartShooter(speed), 0.5);
	AddSequential(new ScriptIndex(0.65, timeout));
	AddSequential(new ScriptStopShooter());
}
