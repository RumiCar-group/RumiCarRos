#include <rumi_hw/PWM.hpp>

#include <rumi_hw/Beeper.hpp>


Beeper::Beeper(RumiPwm& pwm, int index)
    : pwm(pwm)
    , index(index)
{
	if (index < 0) return;

	pwm.setFrequency(index, 0);
	pwm.setDuty(index, 0);
	pwm.toggle(index, true);
}

void Beeper::beep(double frequency, double volume)
{
	if (index < 0) return;

	pwm.setFrequency(index, frequency);
	pwm.setDuty(index, volume);
}
