#pragma once

#include <atomic>
#include <deque>
#include <string>

#include <boost/circular_buffer.hpp>
#include <gpiod.h>

class HallSensor
{
	double noiseRate;
	gpiod_chip* chip = nullptr;
	gpiod_line* line = nullptr;
	boost::circular_buffer<double> onOffTimes;
	std::atomic_bool isRunning = false;
	std::atomic<double> lastTime = 0;
	std::atomic<double> period = 0;

public:
	/**
	 * @param chipName  like "gpiochip0"
	 * @param pin       pin number
	 * @param noiseRate noise rate relative to period
	 */
	HallSensor(const std::string& chipName, int pin, double noiseRate = 0.2);
	~HallSensor();

	/**
	 * @return last period or 0 if stopped
	 */
	double getPeriod() const;

	/**
	 * Block the thread to listen for pin events.
	 */
	void run();

	/**
	 * Stop running.
	 */
	void stop();

private:
	std::pair<int, double> readEvent();
	void handleNewEvent(double time);
};