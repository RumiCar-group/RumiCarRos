#include <cmath>

#include <rumi_hw/GPIO.hpp>
#include <rumi_hw/PWM.hpp>

#include <rumi_hw/DRV8835.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double X_STOP_VELOCITY = 0.01;   // m/s
constexpr double A_STOP_VELOCITY = 0.01;   // rad/s
constexpr double X_ACCELERATION = 1.0;     // m/s/s
constexpr double WHEEL_BASE = 0.11;        // m
constexpr double MIN_CIRCLE_RADIUS = 0.5;  // m
constexpr double STEERING_SPEED = 2.0;     // rad/s

constexpr double STEERING_LIMIT = WHEEL_BASE / MIN_CIRCLE_RADIUS;  // rad

/// @param x > 0
constexpr double dutyFromVelocity(double x)
{
	// https://mycurvefit.com/
	return 0.2 + 1.7 / (1 + std::pow(1.53 / x, 1.73));  // need to make this configurable
}
}  // namespace

DRV8835::DRV8835(
        RumiGpio& gpio, RumiPwm& pwm, std::vector<int> inGpioPins, std::vector<int> inPwmIds, int drivePwmFrequency)
    : gpio(gpio)
    , pwm(pwm)
    , gpioPins(std::move(inGpioPins))
    , pwmIds(std::move(inPwmIds))
    , lastEstimationTime(std::chrono::system_clock::now())
    , lastTwistTime(std::chrono::system_clock::now())
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

DRV8835::Odometry DRV8835::estimateOdometry()
{
	auto now = std::chrono::system_clock::now();
	double t = std::chrono::duration<double>(now - lastEstimationTime).count();
	double xDiff = lastTwist.v - odometry.v;
	double steeringDiff = (lastTwist.a == 0 ? 0 : lastTwist.a < 0 ? -STEERING_LIMIT : STEERING_LIMIT) - steeringAngle;
	steeringAngle += (steeringDiff < 0 ? -1 : 1) * std::min(std::abs(steeringDiff), STEERING_SPEED * t);
	double circleRadius = WHEEL_BASE / std::tan(steeringAngle);

	odometry.v += (xDiff < 0 ? -1 : 1) * std::min(std::abs(xDiff), X_ACCELERATION * t);
	odometry.a = odometry.v / circleRadius;
	odometry.yaw += odometry.a * t;
	odometry.x += odometry.v * std::cos(odometry.yaw) * t;
	odometry.y += odometry.v * std::sin(odometry.yaw) * t;

	lastEstimationTime = now;
	return odometry;
}

void DRV8835::update()
{
	if (std::chrono::system_clock::now() - lastTwistTime > 0.8s)
	{
		drive(0, 0);
	}
}

void DRV8835::drive(double x, double a)
{
	lastTwistTime = std::chrono::system_clock::now();
	lastTwist = {x, a};

	return phaseEnableMode ? drivePhEn(x, a) : driveInIn(x, a);
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
