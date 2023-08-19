#pragma once

class RumiPwm;

/**
 * Beeper control with PWM.
 */
class Beeper
{
    RumiPwm& pwm;
    int index;

public:
    /**
     * @param pwm pwm device
     * @param index pwm index
     */
    Beeper(RumiPwm& pwm, int index);

    /**
     * @param frequency sound frequency (>10 Hz)
     * @param volume sound volume (0.0 ~ 1.0)
     */
    void beep(double frequency, double volume);
};
