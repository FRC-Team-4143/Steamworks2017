#pragma once

#include <Commands/Subsystem.h>
#include "Modules/AnalogChannelVolt.h"
#include "Subsystems/GyroSub.h"

// ==========================================================================

class DriveTrain : public frc::Subsystem {
private:
	static constexpr double pi = 3.14159265358979323846264;

	bool unwinding;

	// Steering Variables
	double X; // X is the distance form the mid-point of the back wheels to the mid-point of the front wheels
	double Y; // Y is the distance from the mid-point of one front wheel to the mid-point of the other front wheel
	//double Z; // Z, distance form A * X to center of rotation

	double FLOffset; // Front left wheel position
	double FROffset; // Front right wheel position
	double RLOffset; // Rear left wheel position
	double RROffset; // Rear right wheel position

	// Inversion Variables
	int FLInv; // Direction of front left drive motor
	int FRInv; // Direction of front right drive motor
	int RLInv; // Direction of right left drive motor
	int RRInv; // Direction of right rear drive motor

	float lastx;
	float lasty;
	float lasttwist;

	double FLValue; // Current value measured from front left drive encoder
	double FRValue; // Current value measured from front right drive encoder
	double RLValue; // Current value measured from rear left drive encoder
	double RRValue; // Current value measured from rear right drive encoder

	//PIDController* frontLeft;			// PID object for front left steering motor
	CANTalon* frontLeftDrive;			// Speed controller for front left drive motor
	//AnalogChannelVolt* frontLeftPos;	// Encoder for front left steering motor
	CANTalon* frontLeftSteer;			// Speed controller for front left steering motor

	//PIDController* frontRight;			// PID object for front right steering motor
	CANTalon* frontRightDrive;			// Speed controller for front right drive motor
	//AnalogChannelVolt* frontRightPos;	// Encoder for front right steering motor
	CANTalon* frontRightSteer;			// Speed controller for front right steering motor

	//PIDController* rearLeft;			// PID object for rear left steering motor
	CANTalon* rearLeftDrive;			// Speed controller for rear left drive motor
	//AnalogChannelVolt* rearLeftPos;		// Encoder for rear left steering motor
	CANTalon* rearLeftSteer;			// Speed controller for rear left steering motor

	//PIDController* rearRight;			// PID object for rear right steering motor
	CANTalon* rearRightDrive;			// Speed controller for rear right drive motor
	//AnalogChannelVolt* rearRightPos;	// Encoder for rear right steering motor
	CANTalon* rearRightSteer;			// Speed controller for rear right steering motor

	double CorrectSteerSetpoint(double setpoint);
	void SetSteerSetpoint(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint);
	void SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed);
	void CurrentLimit();
	double getTalonPosition(int talon);

public:
	DriveTrain();

	void EnablePIDs(bool enable);

	void SetWheelbase(float w, float x, float y); // sets dimensions of drive train
	void SetMaxSpeed(float MaxSpeed); // sets the max speed of drive motors
	void SetOffsets(double FLOff, double FROff, double RLOff, double RROff); // sets wheels to correct positions
	void InitDefaultCommand(); // sets default command
	void Crab(float twist, float y, float x, bool operatorControl); // default driving command
	void Lock();
	void SideLock(); // locks steering
	bool unwind();     // float y, float x); //unwinds all wheels
	void doneunwind(); // called when unwinding is finished
	void GyroCrab(float desiredangle, float y, float x, bool operatorControl); // drive using gyro values
	void FieldCentricCrab(float twist, float y, float x, bool operatorControl); // drive at 90 degree angle to field
	void PositionModeTwist(float desiredangle);
	void updateDistanceEncoders(); // updates the values of drive train variables with distance encoder values
	double getDistanceEncodersValues(); // returns average value from driving encoders
	void zeroDistanceEncoders(); // Reset driving encoders to 0
	void zeroSteeringEncoders(); // Reset steering encoders to 0
	void setWheelOffsets();
	void loadWheelOffsets();
	void LogSettings(double fl, double fr, double rl, double rr);
	void Dashboard();
	void CrabInit();
	void SetWheelsStraight();
	void ArcadeDriveMode(float y, float x);
	void disableSpeedControl();
	void enableSpeedControl();
	void enablePositionControl();
	void enableSteeringPID();
};

// ==========================================================================
