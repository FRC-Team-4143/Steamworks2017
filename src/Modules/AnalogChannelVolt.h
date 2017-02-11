#pragma once

#include <AnalogInput.h>
#include <CANTalon.h>

// ==========================================================================

class AnalogChannelVolt : public frc::AnalogInput {
public:
	AnalogChannelVolt(uint32_t channel, CANTalon* motor, double offset);
	AnalogChannelVolt(uint32_t channel, bool inv, int ratio, CANTalon* motor);
	virtual ~AnalogChannelVolt();

	// PIDSource methods
	double PIDGet() override;

	double GetAngle();
	double GetRawAngle();
	double GetTurns() const;
	void SetOffset(double offset);

private:
	CANTalon* m_motor;
	double m_offset;
};

// ==========================================================================
