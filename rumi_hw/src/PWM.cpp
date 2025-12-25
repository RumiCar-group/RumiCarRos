#include <fstream>
#include <filesystem>

#include <rumi_hw/PWM.hpp>

const std::string PREFIX = "/sys/class/pwm/pwmchip0/";

void throw_if_no_pwm(int ch)
{
	auto file = PREFIX + "pwm" + std::to_string(ch);
	if (!std::filesystem::exists(file))
	{
		throw std::runtime_error("No pwm file: " + file);
	}
}

void write(const std::string& file, int value)
{
	std::ofstream sysFile(PREFIX + file);
	sysFile << std::to_string(value);
}

void checkIndex(int index)
{
	if (index != 0 && index != 1)
		throw std::runtime_error("Wrong pwm index:" + std::to_string(index) + " (must be either 0 or 1).");
}

RumiPwm::RumiPwm()
{
	throw_if_no_pwm(0);
	throw_if_no_pwm(1);
}

void RumiPwm::setFrequency(int index, double value)
{
	checkIndex(index);
	periods[index] = value < 10 ? 0 : static_cast<int>(1'000'000'000 / value);
	write("pwm" + std::to_string(index) + "/period", periods[index]);
}

void RumiPwm::setDuty(int index, double value) const
{
	checkIndex(index);
	if (value > 1.0) value = 1.0;
	if (value < 0.0) value = 0.0;

	int duty = periods[index] ? static_cast<int>(periods[index] * value) : 0;
	write("pwm" + std::to_string(index) + "/duty_cycle", duty);
}

void RumiPwm::toggle(int index, bool on) const
{
	checkIndex(index);
	write("pwm" + std::to_string(index) + "/enable", on ? 1 : 0);
}
