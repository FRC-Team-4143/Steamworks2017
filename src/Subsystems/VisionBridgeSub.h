#pragma once

#include <Commands/Subsystem.h>
#include <mutex>
#include <thread>

// ==========================================================================

class VisionBridgeSub : public frc::Subsystem {
public:
	// Initialize the subsystem with the default UDP port.
	VisionBridgeSub();

	// Initialize the subsystem, specifying the UDP port.
	VisionBridgeSub(uint16_t listeningPort);

	void InitDefaultCommand();

	// Enable/disable debug mode. When debug is enabled,
	// received packets are written to standard output.
	void EnableDebug(bool debug);

	// Get the latest vision data.
	double GetGearPosition();
	double GetGearDistance();
	double GetBoilerPosition();
	double GetBoilerDistance();

private:
	std::recursive_mutex _mutex;
	uint16_t _listeningPort;

	double _gearPosition;
	double _gearDistance;
	double _boilerPosition;
	double _boilerDistance;

	int _zeroCounterGearPos;
	int _zeroCounterGearDist;
	int _zeroCounterBoilerPos;
	int _zeroCounterBoilerDist;

	bool _debug;
	std::thread _listeningThread;

	double _autoAim;

	void DebugOutput(std::string packet);
	void Listen();
	void ParsePacket(char packet[]);
	void SetGearPosition(double position);
	void SetGearDistance(double distance);
	void SetBoilerPosition(double position);
	void SetBoilerDistance(double distance);
};

// ==========================================================================
