#pragma once

/**
 * Simple PWM control based on sysfs.
 */
class RumiPwm final
{
	int periods[2] = {};

public:
	/// Creates 2 PWMs: 0 and 1
	RumiPwm();

	/// PWM frequency in Hz >10 (lower value makes it easier to start moving)
	void setFrequency(int index, double value);

	/// PWM duty 0.0 ~ 1.0
	void setDuty(int index, double value) const;

	/// Toggle on/off
	void toggle(int index, bool on) const;
};
