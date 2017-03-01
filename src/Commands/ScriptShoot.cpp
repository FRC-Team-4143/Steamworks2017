#include "ScriptShoot.h"
#include "ScriptStartShooter.h"
#include "ScriptStopShooter.h"
#include "ScriptIndex.h"

ScriptShoot::ScriptShoot(int speed, float timeout) {
	//Aim
	AddSequential(new ScriptStartShooter(speed));
	AddSequential(new ScriptIndex(0.65, timeout));
	AddSequential(new ScriptStopShooter());
}
