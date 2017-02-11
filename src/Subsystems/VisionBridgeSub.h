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
	double GetPosition(int side);
	double GetDistance();

private:
	std::recursive_mutex _mutex;
	uint16_t _listeningPort;
	double _positionLeft;
	double _positionRight;
	double _position;
	double _distance;
	int _distanceZeroCounter;
	bool _debug;
	std::thread _listeningThread;
	int _zeroCounterLeft;
	int _zeroCounterRight;
	double _autoAim;

	void DebugOutput(std::string packet);
	void Listen();
	void ParsePacket(char packet[]);
	void SetPositionLeft(double position);
	void SetPositionRight(double position);
	void SetDistance(double distance);
};

// ==========================================================================
