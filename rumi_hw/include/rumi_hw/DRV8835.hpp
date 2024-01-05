#pragma once

#include <chrono>
#include <stdexcept>
#include <vector>


class RumiGpio;
class RumiPwm;

/**
 * Motor driver allowing to connect 2 brush motors.
 * It has 2 modes: Phase-Enable and In-In.
 *   Phase - positive (1) or negative (0).
 *   Enable - signal strength (pwm).
 *   In1 - signal strength on positive pin (pwm).
 *   In2 - signal strength on negative pin (pwm).
 */
class DRV8835
{
	using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

	enum { LR_SELECT, STEER_ON, DRIVE_ON, FORWARD_ON, LEFT = 0, RIGHT = 1 };
	enum { DRIVE, DRIVE_FORWARD = 0, DRIVE_BACKWARD = 1 };
	struct Twist
	{
		double v, a;
	};
	struct Odometry : Twist
	{
		double x, y, yaw;
	};

	RumiGpio& gpio;
	RumiPwm& pwm;
	const std::vector<int> gpioPins;
	const std::vector<int> pwmIds;
	bool phaseEnableMode = true;
	TimePoint lastEstimationTime;
	TimePoint lastTwistTime;
	Twist lastTwist = {0, 0};
	double steeringAngle = 0;
	Odometry odometry = {0, 0, 0, 0, 0};

public:
	/**
     * If inPwmIds is only 1 then it's Phase-Enable, if 2 then In-In.
     * @param gpio GPIO device
     * @param pwm PWM device
     * @param inGpioPins Phase-Enable: { LR_SELECT, STEER_ON, DRIVE_ON, FORWARD_ON }
     * @param inGpioPins In-In: { LEFT, RIGHT }
     * @param inPwmIds Phase-Enable: { DRIVE }
     * @param inPwmIds In-In: { DRIVE_FORWARD, DRIVE_BACKWARD }
     * @param drivePwmFrequency [Hz] PWM frequency
     */
	DRV8835(RumiGpio& gpio, RumiPwm& pwm, std::vector<int> inGpioPins, std::vector<int> inPwmIds,
	        int drivePwmFrequency);
	DRV8835(const DRV8835&) = delete;
	~DRV8835();

	/**
	 * @return odometry estimated basing on drive command
	 */
	Odometry estimateOdometry();

	/**
	 * Allows stopping robot if no drive command is received for some time.
	 */
	void update();

	/**
     * @param x [m/s] robot velocity
     * @param a [%] left angle in % of max angle (right angle if negative)
     */
	void drive(double x, double a);

private:
	void drivePhEn(double x, double a);
	void driveInIn(double x, double a);
};
