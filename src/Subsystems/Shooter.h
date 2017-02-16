#pragma once

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <CANTalon.h>

// ==========================================================================

class Shooter : public frc::Subsystem {
public:
	Shooter();

	// Subsystem methods
	virtual void InitDefaultCommand() override;


	void AddMaster(std::shared_ptr<CANTalon> master, bool invertSensor);
	void AddSlave(std::shared_ptr<CANTalon> slave, bool invertDirection, std::shared_ptr<CANTalon> master);

	bool AtTargetRpm() const;
	double GetCurrentRpm() const;
	double GetDeltaRpm() const;
	double GetMaxRpm() const;
	double GetTargetRpm() const;
	double GetToleranceRpm() const;

	void SetDeltaRpm(double deltaRpm);
	void SetMaxRpm(double maxRpm);
	void SetTargetRpm(double targetRpm);
	void SetToleranceRpm(double toleranceRpm);

	void DisableReporting();
	void EnableReporting();
	void SetReportingKey(std::string key);
	void Report();

	void DecreaseTargetRpm();
	void IncreaseTargetRpm();
	void SetSpeed(double speed); // 0.0 to 1.0 as a fraction of maximum RPM

	void Start();
	void Stop();

protected:
	bool _IsRunning() const;
	bool _ReportingEnabled() const;
	void _SetSetpoint(double targetRpm);

	bool _IsMaster(std::shared_ptr<CANTalon> motor) const;
	bool _IsSlave(std::shared_ptr<CANTalon> motor) const;

	static void _ConfigureMaster(CANTalon& motor, bool invertSensor);
	static void _ConfigureSlave(CANTalon& motor, bool invertDirection, CANTalon& master);
	static void _SetMotor(CANTalon& motor, double value);
	static void _StartMotor(CANTalon& motor, double targetRpm);
	static void _StopMotor(CANTalon& motor);

private:
	std::vector<std::shared_ptr<CANTalon>> _masterMotors;
	std::vector<std::shared_ptr<CANTalon>> _slaveMotors;
	bool _isRunning;
	bool _reportingEnabled;
	std::string _reportingKey;
	double _deltaRpm;
	double _maxRpm;
	double _targetRpm;
	double _toleranceRpm;
};

// ==========================================================================
