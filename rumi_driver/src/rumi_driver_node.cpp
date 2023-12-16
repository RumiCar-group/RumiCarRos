#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>

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
    rclcpp::Time lastCommandTime;

    RumiGpio gpio;
    RumiPwm pwm;
    MCP3002 batterySpi;
    DRV8835 driveController;
    Beeper beeper;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPublisher;
    rclcpp::TimerBase::SharedPtr pulseTimer;

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
                    [this](const geometry_msgs::msg::Twist& message) { onTwist(message); }));
        subscribers.push_back(create_subscription<std_msgs::msg::Float32>("beep", 1,
                    [this](const std_msgs::msg::Float32& message) { onBeep(message); }));
        batteryPublisher = create_publisher<sensor_msgs::msg::BatteryState>("battery", 1);

        if (!batteryLedPins.empty())
        {
            pulseTimer = create_wall_timer(1s, [this] { onPulseTimer(); });
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

    void onPulseTimer()
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

        if (now() - lastCommandTime > 0.5s)
        {
            driveController.drive(0, 0);
        }
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

    void onTwist(const geometry_msgs::msg::Twist& message)
    try
    {
        driveController.drive(message.linear.x, message.angular.z);
        lastCommandTime = now();
    }
    catch (const std::exception& ex) {}

    void onBeep(const std_msgs::msg::Float32& message)
    try
    {
        beeper.beep(message.data, message.data == 0 ? 0 : 0.1);
    }
    catch (const std::exception& ex) {}
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RumiDriver>());
    rclcpp::shutdown();
}
