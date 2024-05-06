#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rclcpp/rclcpp.hpp>

cv::Point2d projectionOnLineSegment(const cv::Point& point, const cv::Vec4i& line)
{
	auto p1 = cv::Point(line[0], line[1]), p2 = cv::Point(line[2], line[3]);
	double len = cv::norm(p2 - p1);
	if (len == 0) return p1;

	double t = ((point.x - p1.x) * (p2.x - p1.x) + (point.y - p1.y) * (p2.y - p1.y)) / (len * len);
	t = std::clamp(t, 0.0, 1.0);

	cv::Point2d projection = p1 + t * (p2 - p1);
	return projection;
}

double lineSegmentDistance(const cv::Point& point, const cv::Vec4i& line)
{
	auto projection = projectionOnLineSegment(point, line);
	return cv::norm(cv::Point2d(point) - projection);
}

class RumiLane : public rclcpp::Node
{
	struct Params
	{
		double min_len;
		double line_gap;
		double dist_stop;
		double dist_go;
		double velocity;
		double angular_velocity;
	} params;

	image_transport::CameraSubscriber imageSubscriber;
	OnSetParametersCallbackHandle::SharedPtr paramSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;
	image_transport::CameraPublisher debugImagePublisher;

public:
	RumiLane()
	    : Node("rumi_lane")
	    , params({
	              .min_len = declare_parameter("min_len", 50.0),
	              .line_gap = declare_parameter("line_gap", 10.0),
	              .dist_stop = declare_parameter("dist_stop", 30.0),
	              .dist_go = declare_parameter("dist_go", 80.0),
	              .velocity = declare_parameter("velocity", 0.2),
	              .angular_velocity = declare_parameter("angular_velocity", 0.45),
	      })
	    , imageSubscriber(image_transport::create_camera_subscription(
	              this, "image_raw", [this](const auto& message, const auto& info) { onImage(message, info); }, "raw"))
	    , paramSubscriber(add_on_set_parameters_callback(
	              [this](const std::vector<rclcpp::Parameter>& parameters) { return onParameters(parameters); }))
	    , twistPublisher(create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1))
	    , debugImagePublisher(image_transport::create_camera_publisher(this, "~/image_debug"))
	{
		RCLCPP_INFO(get_logger(), "Initialized");
	}

private:
	void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& message,
	        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
	{
		auto cvImage = getCvImage(message);
		if (!cvImage) return;

		const auto& image = cvImage->image;
		std::vector<cv::Vec4i> lines;

		// Find lines.
		cv::Canny(image, image, 50, 150);
		HoughLinesP(image, lines, 1.0, CV_PI / 180, 50, params.min_len, params.line_gap);

		// Select closest line to bottom center.
		cv::Point origin = {image.cols / 2, image.rows};
		auto bestLine = findClosestLine(origin, lines);
		if (!bestLine) return;

		// Send cmd_Vel
		sendVelocity(origin, *bestLine);

		if (debugImagePublisher.getNumSubscribers())
		{
			const auto& line = *bestLine;
			auto p1 = cv::Point(line[0], line[1]), p2 = cv::Point(line[2], line[3]);
			cv::line(image, p1, p2, 150, 3, cv::LINE_4);
			debugImagePublisher.publish(*cvImage->toImageMsg(), *info);
		}
	}

	cv_bridge::CvImageConstPtr getCvImage(const sensor_msgs::msg::Image::ConstSharedPtr& message)
	try
	{
		return cv_bridge::toCvShare(message);
	}
	catch (cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
		return {};
	}

	static const cv::Vec4i* findClosestLine(const cv::Point& point, const std::vector<cv::Vec4i>& lines)
	{
		double minDist = std::numeric_limits<double>::infinity();
		const cv::Vec4i* bestLine = nullptr;
		for (const auto& line : lines)
		{
			double dist = lineSegmentDistance(point, line);
			if (dist < minDist)
			{
				minDist = dist;
				bestLine = &line;
			}
		}
		return bestLine;
	}

	void sendVelocity(const cv::Point& origin, const cv::Vec4i& line)
	{
		auto projection = projectionOnLineSegment(origin, line);
		auto diff = projection - cv::Point2d(origin);
		double dist = cv::norm(diff);

		geometry_msgs::msg::Twist twist;
		if (dist < params.dist_stop)  // stop
		{
			twist.linear.x = 0;
			twist.angular.z = 0;
		}
		else if (dist > params.dist_go)  // straight
		{
			twist.linear.x = params.velocity;
			twist.angular.z = 0;
		}
		else  // turn
		{
			twist.linear.x = params.velocity;
			twist.angular.z = params.angular_velocity * (diff.x < 0 ? -1 : 1);
		}
		twistPublisher->publish(twist);
	}

	rcl_interfaces::msg::SetParametersResult onParameters(const std::vector<rclcpp::Parameter>& parameters)
	{
		rcl_interfaces::msg::SetParametersResult result;
		for (const auto& parameter : parameters)
		{
			const auto& name = parameter.get_name();
			if (name == "min_len")
				params.min_len = parameter.as_double();
			else if (name == "line_gap")
				params.line_gap = parameter.as_double();
			else if (name == "dist_stop")
				params.dist_stop = parameter.as_double();
			else if (name == "dist_go")
				params.dist_go = parameter.as_double();
			else if (name == "velocity")
				params.velocity = parameter.as_double();
			else if (name == "angular_velocity")
				params.angular_velocity = parameter.as_double();
			else
				break;

			RCLCPP_INFO_STREAM(get_logger(), "Parameter changed: " << name);
		}
		result.successful = true;
		return result;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RumiLane>());
	rclcpp::shutdown();
}
