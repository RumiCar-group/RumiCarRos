#include <rumi_hw/GPIO.hpp>
#include <rumi_hw/PWM.hpp>

#include <rumi_hw/DRV8835.hpp>


constexpr double X_THRESHOLD = 0.15;
constexpr double A_THRESHOLD = 0.2;

DRV8835::DRV8835(RumiGpio& gpio,
                 RumiPwm& pwm,
                 std::vector<int> inGpioPins,
                 std::vector<int> inPwmIds,
                 int drivePwmFrequency)
    : gpio(gpio), pwm(pwm), gpioPins(std::move(inGpioPins)), pwmIds(std::move(inPwmIds))
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
    if (x < -X_THRESHOLD)
    {
        pwm.setDuty(pwmIds[DRIVE], -x);
        gpio.togglePin(gpioPins[FORWARD_ON], false);
    }
    else if (x > X_THRESHOLD)
    {
        pwm.setDuty(pwmIds[DRIVE], x);
        gpio.togglePin(gpioPins[FORWARD_ON], true);
    }
    else
    {
        pwm.setDuty(pwmIds[DRIVE], 0);
    }

    if (a < -A_THRESHOLD)
    {
        gpio.togglePin(gpioPins[LR_SELECT], false);
        gpio.togglePin(gpioPins[STEER_ON], true);
    }
    else if (a > A_THRESHOLD)
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
    if (x < -X_THRESHOLD)
    {
        pwm.setDuty(pwmIds[DRIVE_FORWARD], 0);
        pwm.setDuty(pwmIds[DRIVE_BACKWARD], -x);
    }
    else if (x > X_THRESHOLD)
    {
        pwm.setDuty(pwmIds[DRIVE_FORWARD], x);
        pwm.setDuty(pwmIds[DRIVE_BACKWARD], 0);
    }
    else
    {
        pwm.setDuty(pwmIds[DRIVE_FORWARD], 1);
        pwm.setDuty(pwmIds[DRIVE_BACKWARD], 1);
    }

    if (a < -A_THRESHOLD)
    {
        gpio.togglePin(gpioPins[LEFT], true);
        gpio.togglePin(gpioPins[RIGHT], false);
    }
    else if (a > A_THRESHOLD)
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
