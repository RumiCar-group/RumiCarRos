#include <fstream>
#include <unistd.h>

#include <rumi_pwm.hpp>

const std::string prefix = "/sys/class/pwm/pwmchip0/";

void write(const std::string& file, int value)
{
    std::ofstream sysFile(prefix + file);
    sysFile << std::to_string(value);
}

RumiPwm::RumiPwm()
{
    write("export", 0);
    write("export", 1);

    usleep(100'000);  // uncertain way to know that pin created
}

RumiPwm::~RumiPwm()
{
    write("unexport", 0);
    write("unexport", 1);
}

void RumiPwm::setFrequency(int index, int value)
{
    if (value < 10) return;

    period = 1'000'000'000 / value;
    write("pwm" + std::to_string(index) + "/period", period);
}

void RumiPwm::setDuty(int index, double value) const
{
    if (value > 1.0 || value < 0.0 || period == 0) return;

    int duty = int(period * value);
    write("pwm" + std::to_string(index) + "/duty_cycle", duty);
}

void RumiPwm::toggle(int index, bool on) const
{
    write("pwm" + std::to_string(index) + "/enable", on ? 1 : 0);
}
