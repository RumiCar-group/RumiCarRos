#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

constexpr double SHORT_DIST = 0.3;  // depends on velocity
constexpr double LONG_DIST = 0.6;   // turning radius around 1 m
constexpr double FORWARD_VEL = 0.4;
constexpr double BACK_VEL = 0.5;

class RumiNav : public rclcpp::Node
{
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;

public:
    RumiNav() : Node("rumi_nav")
    {
        subscribers.push_back(create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(),
                    [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& message) { onScan(message); }));
        velocityPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    }

private:
    void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& message)
    try
    {
        float angle = message->angle_min;
        bool isFrontObstacle = false, isRearObstacle = false;
        float leftRange = std::numeric_limits<float>::infinity(), rightRange = leftRange;
        for (float range : message->ranges)
        {
            angle += message->angle_increment;
            if (range == 0.0) continue;  // infinity

            if (!isFrontObstacle && (angle < toRadians(30) && angle >= toRadians(0))) {
                if (range < SHORT_DIST) {
                    isFrontObstacle = true;
                } else if (range < LONG_DIST) {
                    rightRange = std::min(rightRange, range);
                }
            } else if (!isFrontObstacle && (angle < toRadians(0) && angle > toRadians(-30))) {
                if (range < SHORT_DIST) {
                    isFrontObstacle = true;
                } else if (range < LONG_DIST) {
                    leftRange = std::min(leftRange, range);
                }
            } else if (!isRearObstacle && (angle < toRadians(-150) || angle > toRadians(150))) {
                if (range < SHORT_DIST) {
                    isRearObstacle = true;
                }
            }
        }
        geometry_msgs::msg::Twist twist;
        twist.linear.x = isFrontObstacle && isRearObstacle ? 0 : isFrontObstacle ? -BACK_VEL : FORWARD_VEL;
        twist.angular.z = (leftRange == rightRange || twist.linear.x <= 0) ? 0 : leftRange < rightRange ? 1.0 : -1.0;
        velocityPublisher->publish(twist);
    }
    catch (const std::exception& ex) {}

    static float toRadians(float degrees) { return degrees * M_PIf / 180; }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RumiNav>());
    rclcpp::shutdown();
}
