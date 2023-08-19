#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <rumi_hw/Beeper.hpp>
#include <rumi_hw/DRV8835.hpp>
#include <rumi_hw/GPIO.hpp>
#include <rumi_hw/MCP3002.hpp>
#include <rumi_hw/PWM.hpp>

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

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPublisher;
    rclcpp::TimerBase::SharedPtr batteryTimer;

public:
    RumiDriver() : Node("rumi_driver")
            , batteryLedPins(declareInts("battery_led_pins", {5, 6, 16, 20, 21}))
            , batteryLedVoltages(declareFloats("battery_led_voltages", {3.0, 3.2, 3.4, 3.6, 3.8, 4.0}))
            , batterySpi(0, 0)
            , driveController(gpio, pwm,
                      declareInts("drive_gpios", {17, 27, 26, 22}),
                      declareInts("drive_pwms", {0}),
                      static_cast<int>(declare_parameter("drive_pwm_frequency", 36)))
            , beeper(pwm, static_cast<int>(declare_parameter("beeper_pwm", 1)))
    {
        subscribers.push_back(create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
                    [this](const geometry_msgs::msg::Twist::ConstSharedPtr& message) { onTwist(message); }));
        subscribers.push_back(create_subscription<sensor_msgs::msg::Joy>("joy", 1,
                    [this](const sensor_msgs::msg::Joy::ConstSharedPtr& message) { onJoy(message); }));
        batteryPublisher = create_publisher<sensor_msgs::msg::BatteryState>("battery", 1);

        if (!batteryLedPins.empty())
        {
            batteryTimer = create_wall_timer(1s, [this] { onBatteryTimer(); });
        }
        if (batteryLedVoltages.size() > 1)
        {
            minVoltage = batteryLedVoltages.front();
            maxVoltage = batteryLedVoltages.back();
        }
        RCLCPP_INFO(get_logger(), "Initialized");
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

    void onBatteryTimer()
    try
    {
        sensor_msgs::msg::BatteryState state;
        state.header.stamp = this->now();
        state.voltage = batterySpi.get();
        state.current = state.charge = state.capacity = state.design_capacity = NAN;
        state.percentage = (state.voltage - minVoltage) / (maxVoltage - minVoltage);
        state.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
        state.present = true;
        batteryPublisher->publish(state);

        updateLED(state.voltage);
    }
    catch (const std::exception& ex) {}

    void updateLED(float voltage)
    {
        int count = 0;
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
        driveController.drive(message->linear.x, message->angular.z);
    }
    catch (const std::exception& ex) {}

    // Added to reduce CPU usage if joystick connected to robot directly.
    void onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr& message)
    {
        auto twist = std::make_shared<geometry_msgs::msg::Twist>();
        twist->linear.x = message->axes[1] * 0.3;  // max 30%
        twist->angular.z = message->axes[3];
        onTwist(twist);

        beeper.beep(100 + 50 * message->axes[5], message->axes[5] < 0.99 ? 0.1 : 0.0);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RumiDriver>());
    rclcpp::shutdown();
}
