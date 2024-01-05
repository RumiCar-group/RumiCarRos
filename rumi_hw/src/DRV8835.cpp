#include <cmath>

#include <rumi_hw/GPIO.hpp>
#include <rumi_hw/PWM.hpp>

#include <rumi_hw/DRV8835.hpp>

namespace
{
constexpr double X_STOP_VELOCITY = 0.01;  // m/s
constexpr double A_STOP_VELOCITY = 0.01;  // rad/s

/// @param x > 0
constexpr double dutyFromVelocity(double x)
{
	// https://mycurvefit.com/
	return 0.19 + 1.2 / (1 + std::pow(1.53 / x, 1.73));  // need to make this configurable
}
}  // namespace

DRV8835::DRV8835(
        RumiGpio& gpio, RumiPwm& pwm, std::vector<int> inGpioPins, std::vector<int> inPwmIds, int drivePwmFrequency)
    : gpio(gpio)
    , pwm(pwm)
    , gpioPins(std::move(inGpioPins))
    , pwmIds(std::move(inPwmIds))
{
	if (pwmIds.size() == 1)
		phaseEnableMode = true;
	else if (pwmIds.size() == 2)
		phaseEnableMode = false;
	else
		throw std::runtime_error("Number of drive pwms is either 1 or 2.");

	if (phaseEnableMode)
	{
		gpio.togglePin(gpioPins[DRIVE_FORWARD], false);
		gpio.togglePin(gpioPins[DRIVE_ON], true);
	}
	for (int id : pwmIds)
	{
		pwm.setFrequency(id, drivePwmFrequency);
		pwm.setDuty(id, 0);
		pwm.toggle(id, true);
	}
}

DRV8835::~DRV8835()
{
	for (auto pin : gpioPins)
	{
		gpio.togglePin(pin, false);
	}
	for (auto id : pwmIds)
	{
		pwm.setDuty(id, 0);
		pwm.toggle(id, false);
	}
}

void DRV8835::drivePhEn(double x, double a)
{
	if (x < -X_STOP_VELOCITY)
	{
		pwm.setDuty(pwmIds[DRIVE], dutyFromVelocity(-x));
		gpio.togglePin(gpioPins[FORWARD_ON], false);
	}
	else if (x > X_STOP_VELOCITY)
	{
		pwm.setDuty(pwmIds[DRIVE], dutyFromVelocity(x));
		gpio.togglePin(gpioPins[FORWARD_ON], true);
	}
	else
	{
		pwm.setDuty(pwmIds[DRIVE], 0);
	}

	if (a < -A_STOP_VELOCITY)
	{
		gpio.togglePin(gpioPins[LR_SELECT], false);
		gpio.togglePin(gpioPins[STEER_ON], true);
	}
	else if (a > A_STOP_VELOCITY)
	{
		gpio.togglePin(gpioPins[LR_SELECT], true);
		gpio.togglePin(gpioPins[STEER_ON], true);
	}
	else
	{
		gpio.togglePin(gpioPins[STEER_ON], false);
	}
}

void DRV8835::driveInIn(double x, double a)
{
	if (x < -X_STOP_VELOCITY)
	{
		pwm.setDuty(pwmIds[DRIVE_FORWARD], 0);
		pwm.setDuty(pwmIds[DRIVE_BACKWARD], dutyFromVelocity(-x));
	}
	else if (x > X_STOP_VELOCITY)
	{
		pwm.setDuty(pwmIds[DRIVE_FORWARD], dutyFromVelocity(x));
		pwm.setDuty(pwmIds[DRIVE_BACKWARD], 0);
	}
	else
	{
		pwm.setDuty(pwmIds[DRIVE_FORWARD], 1);
		pwm.setDuty(pwmIds[DRIVE_BACKWARD], 1);
	}

	if (a < -A_STOP_VELOCITY)
	{
		gpio.togglePin(gpioPins[LEFT], true);
		gpio.togglePin(gpioPins[RIGHT], false);
	}
	else if (a > A_STOP_VELOCITY)
	{
		gpio.togglePin(gpioPins[LEFT], false);
		gpio.togglePin(gpioPins[RIGHT], true);
	}
	else
	{
		gpio.togglePin(gpioPins[LEFT], false);
		gpio.togglePin(gpioPins[RIGHT], false);
	}
}
