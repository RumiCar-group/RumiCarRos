#pragma once

#include <thread>
#include <optional>

class HallSensor;

class VelocitySensor
{
	std::unique_ptr<HallSensor> hall;
	std::unique_ptr<std::thread> hallThread;

public:
	explicit VelocitySensor(int pin);
	~VelocitySensor();

	std::optional<double> getAbsVelocity() const;
};
