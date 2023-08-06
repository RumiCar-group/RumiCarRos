#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <rumi_hw/GPIO.hpp>
#include <rumi_hw/PWM.hpp>
#include <rumi_hw/MCP3002.hpp>

using namespace std::chrono_literals;

class RumiDriver : public rclcpp::Node
{
    RumiGpio gpio;
    RumiPwm pwm;
    MCP3002 batterySpi;
    std::vector<int64_t> steeringPins;
    std::vector<int64_t> batteryLedPins;
    std::vector<double> batteryLedVoltages;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPublisher;
    rclcpp::TimerBase::SharedPtr batteryTimer;

public:
    RumiDriver() : Node("rumi_driver"), batterySpi(0, 0)
    {
        steeringPins = declare_parameter("steering_pins", std::vector<int64_t>({17, 27}));
        batteryLedPins = declare_parameter("battery_led_pins", std::vector<int64_t>({5, 6, 16, 20, 21}));
        batteryLedVoltages = declare_parameter("battery_led_voltages", std::vector<double>({2.8, 3.2, 3.4, 3.6}));
        auto drivePwmFrequency = static_cast<int>(declare_parameter("drive_pwm_frequency", 36));

        for (int i = 0; i < 2; ++i)
        {
            pwm.setFrequency(i, drivePwmFrequency);
            pwm.setDuty(i, 0);
            pwm.toggle(i, true);
        }

        subscribers.push_back(create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
                    [this](const geometry_msgs::msg::Twist::ConstSharedPtr& message) { onTwist(message); }));
        subscribers.push_back(create_subscription<sensor_msgs::msg::Joy>("joy", 1,
                    [this](const sensor_msgs::msg::Joy::ConstSharedPtr& message) { onJoy(message); }));
        batteryPublisher = create_publisher<sensor_msgs::msg::BatteryState>("battery", 1);

        if (!batteryLedPins.empty())
        {
            batteryTimer = create_wall_timer(1s, [this] { onBatteryTimer(); });
        }
        RCLCPP_INFO(get_logger(), "Start");
    }

private:
    void onBatteryTimer()
    try
    {
        sensor_msgs::msg::BatteryState state;
        state.header.stamp = this->now();
        state.voltage = batterySpi.get();
        state.current = state.charge = state.capacity = state.design_capacity = NAN;
        state.percentage = (state.voltage - 2.f) / (4.2f - 2.f);
        state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
        state.present = true;
        batteryPublisher->publish(state);

        updateLED(state.voltage);
    }
    catch (const std::exception& ex) {}

    void updateLED(float voltage)
    {
        int count = 1;
        for (double ledVoltage: batteryLedVoltages)
        {
            if (voltage < ledVoltage) break;
            ++count;
        }
        for (int i = 0; i < batteryLedPins.size(); ++i)
        {
            gpio.togglePin(batteryLedPins[i], count > i);
        }
    }

    void onTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& message)
    try
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
            gpio.togglePin(steeringPins[0], true);
            gpio.togglePin(steeringPins[1], false);
        }
        else if (message->angular.z > 0.2)
        {
            gpio.togglePin(steeringPins[0], false);
            gpio.togglePin(steeringPins[1], true);
        }
        else
        {
            gpio.togglePin(steeringPins[0], false);
            gpio.togglePin(steeringPins[1], false);
        }
    }
    catch (const std::exception& ex) {}

    // Added to reduce CPU usage if joystick connected to robot directly.
    void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& message)
    {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        twist->linear.x = message->axes[1] * 0.3;  // max 30%
        twist->angular.z = message->axes[3];
        onTwist(twist);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RumiDriver>());
    rclcpp::shutdown();
}
