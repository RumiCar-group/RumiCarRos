#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <unistd.h>

#include <rumi_gpio.hpp>
#include <rumi_pwm.hpp>

// DRV8835 pins
constexpr int AIN1 = 17;
constexpr int AIN2 = 27;

// PWM freq
constexpr int BIN_FREQUENCY = 36; //# lower - lower speeds, higher - smooth movement

class RumiDriver : public rclcpp::Node
{
    RumiGpio gpio;
    RumiPwm pwm;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;

public:
    RumiDriver() : Node("rumi_driver")
    {
        for (int i = 0; i < 2; ++i)
        {
            pwm.setFrequency(i, BIN_FREQUENCY);
            pwm.setDuty(i, 0);
            pwm.toggle(i, true);
        }

        subscribers.push_back(create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
                    [this](const geometry_msgs::msg::Twist::ConstSharedPtr& message) { onTwist(message); }));
        subscribers.push_back(create_subscription<sensor_msgs::msg::Joy>("joy", 1,
                    [this](const sensor_msgs::msg::Joy::ConstSharedPtr& message) { onJoy(message); }));
    }

private:
    void onTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& message)
    {
        if (message->linear.x < -0.15)
        {
            pwm.setDuty(0, 0);
            pwm.setDuty(1, -message->linear.x);
        }
        else if (message->linear.x > 0.15)
        {
            pwm.setDuty(0, message->linear.x);
            pwm.setDuty(1, 0);
        }
        else
        {
            pwm.setDuty(0, 0);
            pwm.setDuty(1, 0);
        }

        if (message->angular.z < -0.2)
        {
            gpio.togglePin(AIN1, true);
            gpio.togglePin(AIN2, false);
        }
        else if (message->angular.z > 0.2)
        {
            gpio.togglePin(AIN1, false);
            gpio.togglePin(AIN2, true);
        }
        else
        {
            gpio.togglePin(AIN1, false);
            gpio.togglePin(AIN2, false);
        }
    }

    // Added to reduce CPU usage if joystick connected to robot directly.
    void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& message)
    {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        twist->linear.x = message->axes[1] * 0.3;  // max 30%
        twist->angular.z = message->axes[2];
        onTwist(twist);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RumiDriver>());
    rclcpp::shutdown();
}