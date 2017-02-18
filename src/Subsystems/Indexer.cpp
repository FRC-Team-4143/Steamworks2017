#include "Indexer.h"
#include "../RobotMap.h"

Indexer::Indexer() : Subsystem("Indexer") {

	indexMotor = RobotMap::indexMotor;
	pdp = RobotMap::pdp;
	timer = new Timer();

	indexJammed = false;

	indexMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	indexMotor->SetControlMode(CANSpeedController::kPosition);
	indexMotor->ConfigNominalOutputVoltage(0.0, 0.0);
	indexMotor->ConfigPeakOutputVoltage(9.0, -9.0);
	indexMotor->SetSensorDirection(true);
	indexMotor->SetP(0.8);
	indexMotor->SetI(0.0);
	indexMotor->SetD(0.05);
	indexMotor->SetF(0.0);
}

void Indexer::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Indexer::SpinCW() {
SmartDashboard::PutBoolean("Indexer Jammed", indexJammed);
	if (indexJammed){
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->Set(-0.7);
		if (timer->Get() > 1){
			indexJammed = false;
			timer->Stop();
		}
	} else {
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->Set(0.7);
		if (IsJammed()){
			indexJammed = true;
			timer->Reset();
			timer->Start();
		}
	}



}

void Indexer::SpinCCW() {
	SmartDashboard::PutBoolean("Indexer Jammed", indexJammed);
	if (indexJammed){
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->Set(0.7);
		if (timer->Get() > 1){
			indexJammed = false;
			timer->Stop();
		}
	} else {
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->Set(-0.7);
		if (IsJammed()){
			indexJammed = true;
			timer->Reset();
			timer->Start();
		}
	}


}

void Indexer::SpinBall() {

	indexMotor->SetControlMode(CANSpeedController::kPosition);
	indexMotor->SetPosition(0.0);
	indexMotor->SetSetpoint(1.0);
}
void Indexer::Stop() {
	indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	indexMotor->Set(0);
}

bool Indexer::IsJammed() {

	if(pdp->GetCurrent(14) > 10){
		return true;
	}
	return false;

}

void Indexer::testJamShooter(){
	indexJammed = true;
	timer->Reset();
	timer->Start();
}

void Indexer::readPDP(){
	for (int i = 0; i < 16; i++){
		SmartDashboard::PutNumber("PDP Current " + std::to_string(i), pdp->GetCurrent(i));
	}
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
