#include "Indexer.h"
#include "RobotMap.h"
#include "Robot.h"
#include "../Commands/DefaultIndexer.h"

Indexer::Indexer() : Subsystem("Indexer") {
	indexMotor = RobotMap::indexMotor;
	pdp = RobotMap::pdp;
	timer = new Timer();
	timer->Start();

	_loadingOne = false;
	jamPosition = 0;
	reverseTime = 0;
	indexJammed = false;

	/*
	indexMotor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	indexMotor->SetControlMode(CANSpeedController::kPosition);
	indexMotor->ConfigNominalOutputVoltage(0.0, 0.0);
	indexMotor->ConfigPeakOutputVoltage(9.0, -9.0);
	indexMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
	indexMotor->SetSensorDirection(true);
	indexMotor->SetP(0.8);
	indexMotor->SetI(0.0);
	indexMotor->SetD(0.05);
	indexMotor->SetF(0.0);
*/
	SmartDashboard::PutNumber("Indexer Speed",0.65);
}

void Indexer::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	 SetDefaultCommand(new DefaultIndexer());

}

void Indexer::SpinCW() {
	SmartDashboard::PutBoolean("Indexer Jammed", indexJammed);
	ReadPDP();

	if (indexJammed) {
		indexMotor->Set(ControlMode::Velocity, 0);
		indexMotor->Set(0.5);
		if (timer->Get() > reverseTime) {
			indexJammed = false;
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kLeftRumble, 0);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kRightRumble, 0);
			reverseTime = 0;
		}
	} else {
		indexMotor->Set(ControlMode::PercentOutput, 0);
		indexMotor->Set(-SmartDashboard::GetNumber("Indexer Speed", 0.65));
		if (IsJammed()) {
			RobotMap::i2c->Write(13, 0);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kLeftRumble, 1);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kRightRumble, 1);
			reverseTime = .3;
			indexJammed = true;
			timer->Reset();
		}
	}
}

void Indexer::SpinCCW() {
	SmartDashboard::PutBoolean("Indexer Jammed", indexJammed);
	if (indexJammed) {
		indexMotor->Set(ControlMode::PercentOutput, 0);
		indexMotor->Set(-0.5);
		if (timer->Get() > reverseTime) {
			indexJammed = false;
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kLeftRumble, 0);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kRightRumble, 0);
			reverseTime = 0;
		}
	}
	else {
		indexMotor->Set(ControlMode::PercentOutput, 0);
		indexMotor->Set(SmartDashboard::GetNumber("Indexer Speed", 0.65));
		if (IsJammed()) {
			RobotMap::i2c->Write(13, 0);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kLeftRumble, 1);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kRightRumble, 1);
			reverseTime = .3;
			indexJammed = true;
			timer->Reset();
		}
	}
}

//USED FOR AUTONOMOUS//
void Indexer::SpinCW(float speed){
	if (indexJammed){
		indexMotor->Set(ControlMode::PercentOutput, 0);
		indexMotor->Set(0.5);
		if (timer->Get() > reverseTime){
			indexJammed = false;
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kLeftRumble, 0);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kRightRumble, 0);
		}

	} else {
		indexMotor->Set(ControlMode::PercentOutput, 0);
		indexMotor->Set(-speed);
		if (IsJammed()) {
			RobotMap::i2c->Write(13, 0);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kLeftRumble, 1);
			Robot::oi->GetDriverJoystick2()->SetRumble(Joystick::kRightRumble, 1);
			reverseTime = .3;
			indexJammed = true;
			timer->Reset();
		}
	}

}
//////////////////////

/*void Indexer::SpinBall() {
	indexMotor->Set(ControlMode::PercentOutput, 10);
	indexMotor->SetPosition(0.0);
	indexMotor->SetSetpoint(-1.0);
}*/

void Indexer::SetLoadingOne(bool loadingOne) {
	_loadingOne = loadingOne;
}

void Indexer::Stop() {
	indexMotor->Set(ControlMode::PercentOutput, 0);
	indexMotor->Set(0);
	RobotMap::i2c->Write(6, 0);
}

bool Indexer::IsJammed() {
	if (pdp->GetCurrent(9) > 15) {
		return false;
	}
	return false;
}

void Indexer::TestJamShooter() {
	indexJammed = true;
	reverseTime += .1;
	RobotMap::i2c->Write(13, 0);
	timer->Reset();
	timer->Start();
}

void Indexer::ReadPDP() {
	for (int i = 0; i < 16; i++) {
		SmartDashboard::PutNumber("PDP Current " + std::to_string(i),
				pdp->GetCurrent(i));
	}
}

void Indexer::SetSpeed(double speed) {
	if (!Robot::oi->GetButton6() && !Robot::oi->GetButton10() && !_loadingOne) {
		//indexMotor->SetControlMode(CANTalon::kSpeed);
		indexMotor->Set(ControlMode::Velocity, 10);
		indexMotor->Set(speed);
	}
}

void Indexer::ZeroTimer() {
	reverseTime = 0;
}

double Indexer::GetPosition() const {
	return indexMotor->GetSelectedSensorPosition(0);
}

double Indexer::GetReverseTime() const {
	return reverseTime;
}

double Indexer::GetTimer() const {
	return timer->Get();
}

bool Indexer::IsIndexJammed() const {
	return indexJammed;
}
