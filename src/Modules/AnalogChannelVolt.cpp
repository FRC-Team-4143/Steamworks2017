#include "Modules/AnalogChannelVolt.h"
#include "Subsystems/EncoderConstants.h"

// ==========================================================================

AnalogChannelVolt::AnalogChannelVolt(uint32_t channel, CANTalon *motor, double offset)
:	AnalogInput(channel) {
	m_motor = motor;
	m_offset = offset;
}

// ==========================================================================

AnalogChannelVolt::AnalogChannelVolt(uint32_t channel, bool inv, int ratio, CANTalon* motor)
:	AnalogInput(channel) {
	SetSampleRate(1000);
	m_motor = motor;
	m_offset = 0;
}

// ==========================================================================

AnalogChannelVolt::~AnalogChannelVolt() {
}

// ==========================================================================

double AnalogChannelVolt::PIDGet() {
	auto position = m_motor->GetPosition();
	position -= trunc(position);
	if (position < 0) {
		position++;
	}
	position *= EncoderConstants::FULL_TURN;
	return position;
}

// ==========================================================================

double AnalogChannelVolt::GetAngle() {
	auto position = m_motor->GetPosition() - m_offset;
	position -= trunc(position);
	if (position < 0) {
		position++;
	}
	position *= EncoderConstants::FULL_TURN;
	return position;
}

// ==========================================================================

double AnalogChannelVolt::GetRawAngle() {
	return PIDGet();
}

// ==========================================================================

double AnalogChannelVolt::GetTurns() const {
	return m_motor->GetPosition() - m_offset;
}

// ==========================================================================

void AnalogChannelVolt::SetOffset(double offset) {
	m_offset = offset;
}

// ==========================================================================
