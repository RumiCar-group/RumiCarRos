#pragma once

#include <atomic>
#include <thread>
#include <iostream>

class HallSensor;

class VelocitySensor
{
	std::unique_ptr<HallSensor> hall;
	std::unique_ptr<std::thread> hallThread;

public:
	explicit VelocitySensor(int pin);
	~VelocitySensor();

	double getVelocity() const;
};
