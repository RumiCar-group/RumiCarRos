#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>

class RumiTeleop : public rclcpp::Node
{
	std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr beepPublisher;

	const int axisLinear, axisAngular, axisBeep;
	const float scaleLinear, scaleAngular;

public:
	RumiTeleop()
	    : Node("rumi_teleop")
	    , axisLinear(static_cast<int>(declare_parameter<int>("axis_linear.x")))
	    , axisAngular(static_cast<int>(declare_parameter<int>("axis_angular.yaw")))
	    , axisBeep(static_cast<int>(declare_parameter<int>("axis_beep.tone")))
	    , scaleLinear(static_cast<float>(declare_parameter<float>("scale_linear.x")))
	    , scaleAngular(static_cast<float>(declare_parameter<float>("scale_angular.yaw")))
	{
		subscribers.push_back(create_subscription<sensor_msgs::msg::Joy>(
		        "joy", 1, [this](const sensor_msgs::msg::Joy& message) { onJoy(message); }));
		twistPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
		beepPublisher = create_publisher<std_msgs::msg::Float32>("beep", 1);
	}

private:
	void onJoy(const sensor_msgs::msg::Joy& message)
	{
		geometry_msgs::msg::Twist twist;
		twist.linear.x = message.axes[axisLinear] * scaleLinear;
		twist.angular.z = message.axes[axisAngular] * scaleAngular;
		twistPublisher->publish(twist);

		std_msgs::msg::Float32 beep;
		beep.data = message.axes[axisBeep] < 0.99f ? 100 + 50 * message.axes[axisBeep] : 0.f;
		beepPublisher->publish(beep);
	}
};

int main(int argc, char* argv[])
try
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RumiTeleop>());
	rclcpp::shutdown();
}
catch (std::exception& e)
{
	RCLCPP_FATAL_STREAM(rclcpp::get_logger("main"), e.what());
	return EXIT_FAILURE;
}
