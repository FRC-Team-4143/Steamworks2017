#include "Subsystems/Shooter.h"
#include <CANTalon.h>
#include <Commands/ShooterDefault.h>
#include <Modules/Logger.h>
#include "RobotMap.h"

const double SHOOTER_PID_P = 0.05;
const double SHOOTER_PID_I = 0;
const double SHOOTER_PID_D = 0;
const double SHOOTER_PID_F = 0.03;

const double DEFAULT_DELTA_RPM = 100;
const double DEFAULT_MAX_RPM = 8000;
const double DEFAULT_TARGET_RPM = 4000;
const double DEFAULT_TOLERANCE_RPM = DEFAULT_DELTA_RPM / 2;

const std::string DEFAULT_REPORTING_KEY = "Shooter";

const bool USE_SOFT_LIMITS = true;
const double SOFT_LIMIT_RPM = 12000;

// ==========================================================================

Shooter::Shooter()
:	frc::Subsystem("Shooter"),
	_masterMotors(),
	_slaveMotors(),
	_isRunning(false),
	_reportingEnabled(false),
	_reportingKey(DEFAULT_REPORTING_KEY),
	_deltaRpm(DEFAULT_DELTA_RPM),
	_maxRpm(DEFAULT_MAX_RPM),
	_targetRpm(DEFAULT_TARGET_RPM),
	_toleranceRpm(DEFAULT_TOLERANCE_RPM) {
}

// ==========================================================================

void Shooter::InitDefaultCommand() {
	SetDefaultCommand(new ShooterDefault());
}

// ==========================================================================

void Shooter::AddMaster(std::shared_ptr<CANTalon> master, bool invertSensor) {
	if (!master) {
		throw std::invalid_argument("master");
	}

	if (_IsSlave(master)) {
		throw std::runtime_error("Master is already configured as a slave.");
	}

	if (!_IsMaster(master)) {
		_masterMotors.push_back(master);
	}

	_ConfigureMaster(*master, invertSensor);
}

// ==========================================================================

void Shooter::AddSlave(std::shared_ptr<CANTalon> slave, bool invertDirection, std::shared_ptr<CANTalon> master) {
	if (!slave) {
		throw std::invalid_argument("slave");
	}

	if (!master) {
		throw std::invalid_argument("master");
	}

	if (master == slave) {
		throw std::runtime_error("Slave may not have itself as master.");
	}

	if (_IsMaster(slave)) {
		throw std::runtime_error("Slave is already configured as a master.");
	}

	if (!_IsMaster(master)) {
		throw std::runtime_error("Master is not yet configured.");
	}

	if (!_IsSlave(slave)) {
		_slaveMotors.push_back(slave);
	}

	_ConfigureSlave(*slave, invertDirection, *master);
}

// ==========================================================================

bool Shooter::AtTargetRpm() const {
	auto currentRpm = GetCurrentRpm();
	auto targetRpm = GetTargetRpm();
	auto toleranceRpm = GetToleranceRpm();
	return currentRpm >= (targetRpm - toleranceRpm) && currentRpm <= (targetRpm + toleranceRpm);
}

// ==========================================================================

double Shooter::GetCurrentRpm() const {
	if (_masterMotors.size() > 0) {
		return _masterMotors.begin()->get()->GetSpeed();
	}
	return 0.0;
}

// ==========================================================================

double Shooter::GetDeltaRpm() const {
	return _deltaRpm;
}

// ==========================================================================

double Shooter::GetMaxRpm() const {
	return _maxRpm;
}

// ==========================================================================

double Shooter::GetTargetRpm() const {
	return _targetRpm;
}

// ==========================================================================

double Shooter::GetToleranceRpm() const {
	return _toleranceRpm;
}

// ==========================================================================

void Shooter::SetDeltaRpm(double deltaRpm) {
	_deltaRpm = std::abs(deltaRpm);
}

// ==========================================================================

void Shooter::SetMaxRpm(double maxRpm) {
	_maxRpm = std::abs(maxRpm);
	if (GetTargetRpm() > GetMaxRpm()) {
		SetTargetRpm(GetMaxRpm());
	}
}

// ==========================================================================

void Shooter::SetTargetRpm(double targetRpm) {
	_targetRpm = std::max(std::min(targetRpm, GetMaxRpm()), 0.0);
	_SetSetpoint(GetTargetRpm());
}

// ==========================================================================

void Shooter::SetToleranceRpm(double toleranceRpm) {
	_toleranceRpm = std::abs(toleranceRpm);
}

// ==========================================================================

void Shooter::DisableReporting() {
	_reportingEnabled = false;
}

// ==========================================================================

void Shooter::EnableReporting() {
	_reportingEnabled = true;
}

// ==========================================================================

void Shooter::SetReportingKey(std::string key) {
	_reportingKey = key;
}

// ==========================================================================

void Shooter::Report() {
	if (_ReportingEnabled()) {
		SmartDashboard::PutNumber(_reportingKey + "-CurrentRpm", GetCurrentRpm());
		SmartDashboard::PutNumber(_reportingKey + "-TargetRpm", GetTargetRpm());
	}
}

// ==========================================================================

void Shooter::DecreaseTargetRpm() {
	SetTargetRpm(GetTargetRpm() - GetDeltaRpm());
}

// ==========================================================================

void Shooter::IncreaseTargetRpm() {
	SetTargetRpm(GetTargetRpm() + GetDeltaRpm());
}

// ==========================================================================

void Shooter::SetSpeed(double speed) {
	speed = std::abs(speed);
	speed = std::max(std::min(speed, 1.0), 0.0);
	_SetSetpoint(speed * GetMaxRpm());
}

// ==========================================================================

void Shooter::Start() {
	_isRunning = true;
	auto targetRpm = GetTargetRpm();
	std::for_each(_masterMotors.begin(), _masterMotors.end(), [targetRpm](std::shared_ptr<CANTalon> motor) {
		_StartMotor(*motor, targetRpm);

	});
}

// ==========================================================================

void Shooter::Stop() {
	_isRunning = false;
	std::for_each(_masterMotors.begin(), _masterMotors.end(), [](std::shared_ptr<CANTalon> motor) {
		_StopMotor(*motor);
	});
}

// ==========================================================================

bool Shooter::_IsRunning() const {
	return _isRunning;
}

// ==========================================================================

bool Shooter::_ReportingEnabled() const {
	return _reportingEnabled;
}

// ==========================================================================

void Shooter::_SetSetpoint(double targetRpm) {
	if (_IsRunning()) {
		std::for_each(_masterMotors.begin(), _masterMotors.end(), [targetRpm](std::shared_ptr<CANTalon> motor) {
			_SetMotor(*motor, targetRpm);
		});
	}
}

// ==========================================================================

bool Shooter::_IsMaster(std::shared_ptr<CANTalon> motor) const {
	auto match = std::find(_masterMotors.begin(), _masterMotors.end(), motor);
	auto found = match != _masterMotors.end();
	return found;
}

// ==========================================================================

bool Shooter::_IsSlave(std::shared_ptr<CANTalon> motor) const {
	auto match = std::find(_slaveMotors.begin(), _slaveMotors.end(), motor);
	auto found = match != _slaveMotors.end();
	return found;
}

// ==========================================================================

void Shooter::_ConfigureMaster(CANTalon& motor, bool invertSensor) {
	motor.ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
	motor.ConfigNominalOutputVoltage(0.0, 0.0);
	motor.ConfigPeakOutputVoltage(12.0, -12.0);
	motor.SetVoltageRampRate(24);

	motor.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	motor.SetSensorDirection(invertSensor);

	if (USE_SOFT_LIMITS) {
		motor.ConfigLimitMode(CANSpeedController::kLimitMode_SoftPositionLimits);
		motor.ConfigForwardLimit(SOFT_LIMIT_RPM);
		motor.ConfigReverseLimit(0);
	}
	else {
		motor.ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
	}

	motor.SetControlMode(CANSpeedController::kPercentVbus);
	motor.SetP(SHOOTER_PID_P);
	motor.SetI(SHOOTER_PID_I);
	motor.SetD(SHOOTER_PID_D);
	motor.SetF(SHOOTER_PID_F);
	motor.Set(0);
}

// ==========================================================================

void Shooter::_ConfigureSlave(CANTalon& motor, bool invertDirection, CANTalon& master) {
	motor.ConfigNeutralMode(CANSpeedController::kNeutralMode_Coast);
	motor.ConfigNominalOutputVoltage(0.0, 0.0);
	motor.ConfigPeakOutputVoltage(12.0, -12.0);
	motor.SetVoltageRampRate(24);

	motor.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	motor.SetSensorDirection(false);

	if (USE_SOFT_LIMITS) {
		motor.ConfigLimitMode(CANSpeedController::kLimitMode_SoftPositionLimits);
		motor.ConfigForwardLimit(invertDirection ? 0 : SOFT_LIMIT_RPM);
		motor.ConfigReverseLimit(invertDirection ? -SOFT_LIMIT_RPM : 0);
	}
	else {
		motor.ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
	}

	motor.SetControlMode(CANSpeedController::kFollower);
	motor.SetClosedLoopOutputDirection(invertDirection);
	motor.Set(master.GetDeviceID());
}

// ==========================================================================

void Shooter::_SetMotor(CANTalon& motor, double value) {
	motor.Set(value);
}

// ==========================================================================

void Shooter::_StartMotor(CANTalon& motor, double targetRpm) {
	motor.SetControlMode(CANSpeedController::kSpeed);
	_SetMotor(motor, targetRpm);
}

// ==========================================================================

void Shooter::_StopMotor(CANTalon& motor) {
	motor.SetControlMode(CANSpeedController::kPercentVbus);
	_SetMotor(motor, 0);
}

// ==========================================================================
