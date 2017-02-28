#include "ScriptShoot.h"
#include "ScriptStartShooter.h"
#include "ScriptStopShooter.h"
#include "ScriptIndex.h"

ScriptShoot::ScriptShoot() {
	//Aim
	AddSequential(new ScriptStartShooter(5000));
	AddSequential(new ScriptIndex(0.65, 15));
	AddSequential(new ScriptStopShooter());
}
