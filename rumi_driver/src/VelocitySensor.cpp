#include "VelocitySensor.hpp"

#include <rumi_hw/HallSensor.hpp>

#include <cmath>
#include <rclcpp/logging.hpp>

namespace
{
constexpr int MAGNETS_NUMBER = 2;
constexpr int MOTOR_NUMBER = 10;  // 1st gear teeth
constexpr int WHEEL_NUMBER = 22;  // 2nd gear teeth
constexpr double WHEEL_FACTOR = double(WHEEL_NUMBER) / MOTOR_NUMBER * MAGNETS_NUMBER;
constexpr double WHEEL_DIAMETER = 0.03;   // 30 mm
constexpr double MAX_WHEEL_PERIOD = 2.0;  // [s] max time to do full wheel rotation
}  // namespace

VelocitySensor::VelocitySensor(int pin)
    : hall(pin ? std::make_unique<HallSensor>("gpiochip0", pin) : nullptr)
{
	if (hall)
	{
		hallThread = std::make_unique<std::thread>([this] { hall->run(); });
		RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity"), "Start");
	}
	RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity"), "No sensor");
}

VelocitySensor::~VelocitySensor()
{
	if (hall)
	{
		RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity"), "Stop");
		hall->stop();
	}
	if (hallThread)
	{
		hallThread->join();
		RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity"), "Stopped");
	}
}

double VelocitySensor::getVelocity() const
{
	if (!hall)
	{
		return 0;
	}
	double period = hall->getPeriod();
	RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity"), "Period: " << period << ", freq: " << 1 / period);

	if (period == 0)
	{
		return 0;  // 0 period means sensor gave up on waiting
	}
	double wheelPeriod = period * WHEEL_FACTOR;
	if (wheelPeriod > MAX_WHEEL_PERIOD)
	{
		return 0;  // too big period means too old last triggering
	}
	return M_PI * WHEEL_DIAMETER / wheelPeriod;
}
