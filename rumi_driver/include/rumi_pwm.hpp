#pragma once

/**
 * Simple PWM control based on sysfs.
 */
class RumiPwm final
{
    int period = 0;

public:
    RumiPwm();
    ~RumiPwm();

    /// PWM frequency in Hz >10 (lower value makes it easier to start moving)
    void setFrequency(int index, int value);

    /// PWM duty 0.0 ~ 1.0
    void setDuty(int index, double value) const;

    /// Toggle on/off
    void toggle(int index, bool on) const;
};
