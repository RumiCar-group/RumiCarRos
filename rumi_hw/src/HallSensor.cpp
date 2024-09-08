#include <stdexcept>

#include <rumi_hw/HallSensor.hpp>

namespace
{
constexpr int HALL_TIMEOUT_S = 1;
constexpr int HALL_TIMEOUT_EVENT = -1;

double getSeconds(const timespec& ts)
{
	return double(ts.tv_sec) + double(ts.tv_nsec) / 1e9;
}
double getCurrentSeconds()
{
	timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1)
	{
		return 0;
	}
	return getSeconds(ts);
}
}  // namespace

HallSensor::HallSensor(const std::string& chipName, int pin, double noiseRate)
    : noiseRate(noiseRate)
    , onOffTimes(5)  // on, off, on, off, last
{
	chip = gpiod_chip_open_by_name(chipName.data());
	if (!chip)
	{
		throw std::runtime_error("Failed to open GPIO chip");
	}
	line = gpiod_chip_get_line(chip, pin);
	if (!line)
	{
		throw std::runtime_error("Failed to get GPIO line");
	}
	if (gpiod_line_request_both_edges_events(line, "gpio-event") < 0)
	{
		throw std::runtime_error("Failed to request events on GPIO line");
	}
}

HallSensor::~HallSensor()
{
	if (line) gpiod_line_release(line);
	if (chip) gpiod_chip_close(chip);
}

double HallSensor::getPeriod() const
{
	if (!period)
	{
		return 0;
	}
	double now = getCurrentSeconds();
	return std::max(period.load(), now - lastTime);
}

void HallSensor::run()
{
	int lastEvent = 0;
	isRunning = true;

	while (isRunning)
	{
		auto [event, time] = readEvent();
		if (!event || lastEvent == event)
		{
			continue;
		}
		if (event == HALL_TIMEOUT_EVENT)
		{
			onOffTimes.clear();
			period = 0;
			lastTime = 0;
			continue;
		}
		handleNewEvent(time);
		lastEvent = event;
		lastTime = time;
	}
}

std::pair<int, double> HallSensor::readEvent()
{
	timespec stop_timeout = {HALL_TIMEOUT_S, 0};
	int result = gpiod_line_event_wait(line, &stop_timeout);
	if (result == 0)
	{
		return {HALL_TIMEOUT_EVENT, 0};
	}
	if (result < 0)
	{
		return {};  // Error waiting for event
	}
	gpiod_line_event line_event = {};
	if (gpiod_line_event_read(line, &line_event) < 0)
	{
		return {};  // Error reading event
	}
	int event = line_event.event_type;
	double time = getSeconds(line_event.ts);
	return {event, time};
}

void HallSensor::handleNewEvent(double time)
{
	int N = onOffTimes.size();
	if (onOffTimes.size() == 5)
	{
		double nextPeriod = time - onOffTimes[N - 1];
		double lastPeriod = onOffTimes[N - 1] - onOffTimes[N - 2];
		if (std::max(nextPeriod, lastPeriod) < period * noiseRate)
		{
			// remove noise spike
			onOffTimes.pop_back();
			return;
		}
	}
	onOffTimes.push_back(time);

	N = onOffTimes.size();
	if (N < 5)
	{
		// wait until we have at least 2 measurement pairs on/off and last uncertain one.
		return;
	}
	double aPeriod = onOffTimes[N - 2] - onOffTimes[N - 4];
	double bPeriod = onOffTimes[N - 3] - onOffTimes[N - 5];
	period = (aPeriod + bPeriod) / 2;  // average of raise diff & fall diff
}

void HallSensor::stop()
{
	isRunning = false;
}
