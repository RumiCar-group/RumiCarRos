#include <chrono>
#include <ranges>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <rumi_hw/Beeper.hpp>
#include <rumi_hw/DRV8835.hpp>
#include <rumi_hw/GPIO.hpp>
#include <rumi_hw/MCP3002.hpp>
#include <rumi_hw/PWM.hpp>

#include "VelocitySensor.hpp"

using namespace std::chrono_literals;

class RumiDriver : public rclcpp::Node
{
	std::vector<int> batteryLedPins;
	std::vector<float> batteryLedVoltages;
	float minVoltage = 0, maxVoltage = 1;

	RumiGpio gpio;
	RumiPwm pwm;
	MCP3002 batterySpi;
	DRV8835 driveController;
	Beeper beeper;
	VelocitySensor velocitySensor;

	tf2_ros::TransformBroadcaster odometryBroadcaster;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;
	rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPublisher;
	rclcpp::TimerBase::SharedPtr pulseTimer;

public:
	RumiDriver()
	    : Node("rumi_driver")
	    , batteryLedPins(declareInts("battery_led_pins", {5, 6, 16, 20, 21}))
	    , batteryLedVoltages(declareFloats("battery_led_voltages", {3.0, 3.2, 3.4, 3.6, 3.8, 4.0}))
	    , batterySpi(0, 0)
	    , driveController(gpio, pwm, declareInts("drive_gpios", {17, 27, 26, 22}), declareInts("drive_pwms", {0}),
	              static_cast<int>(declare_parameter("drive_pwm_frequency", 36)))
	    , beeper(pwm, static_cast<int>(declare_parameter("beeper_pwm", 1)))
	    , velocitySensor(static_cast<int>(declare_parameter("velocity_pin", 0)))
	    , odometryBroadcaster(this)
	{
		subscribers.push_back(create_subscription<geometry_msgs::msg::Twist>(
		        "cmd_vel", 1, [this](const geometry_msgs::msg::Twist& message) { onTwist(message); }));
		subscribers.push_back(create_subscription<std_msgs::msg::Float32>(
		        "beep", 1, [this](const std_msgs::msg::Float32& message) { onBeep(message); }));
		odometryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
		batteryPublisher = create_publisher<sensor_msgs::msg::BatteryState>("battery", 1);

		if (!batteryLedPins.empty())
		{
			pulseTimer = create_wall_timer(0.1s, [this] { onPulseTimer(); });
		}
		if (batteryLedVoltages.size() > 1)
		{
			minVoltage = batteryLedVoltages.front();
			maxVoltage = batteryLedVoltages.back();
		}
		RCLCPP_INFO(get_logger(), "=== Parameters ===");
		for (const auto& name: list_parameters({}, 0).names
			| std::views::filter([](auto& s){ return !s.starts_with("qos"); }))
		{
				std::string value = get_parameter(name).value_to_string();
				RCLCPP_INFO_STREAM(get_logger(), name << ": " << value);
		}
	}

	~RumiDriver() override
	{
		for (long batteryLedPin : batteryLedPins)
		{
			gpio.togglePin(batteryLedPin, false);
		}
	}

private:
	std::vector<int> declareInts(const std::string& param, const std::vector<int>& defaults)
	{
		auto ints64 = declare_parameter(param, defaults);
		return {ints64.begin(), ints64.end()};
	}

	std::vector<float> declareFloats(const std::string& param, const std::vector<float>& defaults)
	{
		auto doubles = declare_parameter(param, defaults);
		return {doubles.begin(), doubles.end()};
	}

	void onPulseTimer()
	try
	{
		auto currentTime = now();
		auto odom = driveController.estimateOdometry();
		double velocity = velocitySensor.getVelocity();
		tf2::Quaternion tfYaw;
		tfYaw.setRPY(0, 0, odom.yaw);

		nav_msgs::msg::Odometry odometry;
		odometry.header.stamp = currentTime;
		odometry.header.frame_id = "odom";
		odometry.child_frame_id = "base_footprint";
		odometry.twist.twist.linear.x = velocity * (odom.v < 0 ? -1 : odom.v > 0 ? 1 : 0);
		odometry.twist.twist.angular.z = odom.a;
		odometry.pose.pose.position.x += odom.x;
		odometry.pose.pose.position.y += odom.y;
		odometry.pose.pose.position.z = 0;
		odometry.pose.pose.orientation = tf2::toMsg(tfYaw);
		odometryPublisher->publish(odometry);

		geometry_msgs::msg::TransformStamped tf;
		tf.header.stamp = currentTime;
		tf.header.frame_id = "odom";
		tf.child_frame_id = "base_footprint";
		tf.transform.translation.x = odometry.pose.pose.position.x;
		tf.transform.translation.y = odometry.pose.pose.position.y;
		tf.transform.translation.z = odometry.pose.pose.position.z;
		tf.transform.rotation = odometry.pose.pose.orientation;
		odometryBroadcaster.sendTransform(tf);

		sensor_msgs::msg::BatteryState state;
		state.header.stamp = currentTime;
		state.voltage = batterySpi.get();
		state.current = state.charge = state.capacity = state.design_capacity = NAN;
		state.percentage = (state.voltage - minVoltage) / (maxVoltage - minVoltage);
		state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
		state.present = true;
		batteryPublisher->publish(state);

		updateLED(state.voltage);

		driveController.update();
	}
	catch (const std::exception& ex)
	{}

	void updateLED(float voltage)
	{
		int count = 0;
		for (double ledVoltage : batteryLedVoltages)
		{
			if (voltage < ledVoltage) break;
			++count;
		}
		for (int i = 0; i < batteryLedPins.size(); ++i)
		{
			gpio.togglePin(batteryLedPins[i], count > i);
		}
	}

	void onTwist(const geometry_msgs::msg::Twist& message)
	try
	{
		driveController.drive(message.linear.x, message.angular.z);
	}
	catch (const std::exception& ex)
	{}

	void onBeep(const std_msgs::msg::Float32& message)
	try
	{
		beeper.beep(message.data, message.data == 0 ? 0 : 0.1);
	}
	catch (const std::exception& ex)
	{}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RumiDriver>());
	rclcpp::shutdown();
}
