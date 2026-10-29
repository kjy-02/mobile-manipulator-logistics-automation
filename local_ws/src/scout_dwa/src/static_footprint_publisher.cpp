#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include "footprint_validation.h"

class StaticFootprintPublisher
{
public:
  StaticFootprintPublisher() : private_nh_("~")
  {
    private_nh_.param<std::string>("topic", topic_, "/scout_dwa/footprint");
    private_nh_.param<std::string>("frame_id", frame_id_, "base_link");
    private_nh_.param("publish_rate", publish_rate_, 1.0);
    if (frame_id_.empty() || publish_rate_ <= 0.0)
      throw std::runtime_error("frame_id and publish_rate must be valid");

    XmlRpc::XmlRpcValue footprint;
    if (!private_nh_.getParam("footprint", footprint))
      throw std::runtime_error("required parameter ~footprint is missing");
    parseFootprint(footprint);
    validateFootprint();

    pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(topic_, 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                            &StaticFootprintPublisher::timerCallback, this);
    publish();
  }

private:
  static double number(const XmlRpc::XmlRpcValue& value)
  {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      return static_cast<int>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      return static_cast<double>(value);
    throw std::runtime_error("footprint coordinates must be numeric");
  }

  void parseFootprint(XmlRpc::XmlRpcValue& value)
  {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() < 3)
      throw std::runtime_error("footprint must contain at least three [x, y] points");
    for (int i = 0; i < value.size(); ++i)
    {
      if (value[i].getType() != XmlRpc::XmlRpcValue::TypeArray || value[i].size() != 2)
        throw std::runtime_error("each footprint point must be [x, y]");
      geometry_msgs::Point32 point;
      point.x = number(value[i][0]);
      point.y = number(value[i][1]);
      footprint_.polygon.points.push_back(point);
    }
  }

  void validateFootprint() const
  {
    double radius;
    if (!validateFootprintPolygon(footprint_.polygon.points, radius))
      throw std::runtime_error("footprint must be finite, simple, nondegenerate and contain origin");
    double twice_area = 0.0;
    bool origin_inside = false;
    const auto& points = footprint_.polygon.points;
    for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++)
    {
      twice_area += static_cast<double>(points[j].x) * points[i].y -
                    static_cast<double>(points[i].x) * points[j].y;
      const bool crosses = ((points[i].y > 0.0) != (points[j].y > 0.0)) &&
          (0.0 < (points[j].x - points[i].x) * (-points[i].y) /
                     (points[j].y - points[i].y) + points[i].x);
      if (crosses)
        origin_inside = !origin_inside;
    }
    if (std::fabs(twice_area) < 1e-6)
      throw std::runtime_error("footprint area must be non-zero");
    if (!origin_inside)
      throw std::runtime_error("footprint must contain the base_link origin");
  }

  void timerCallback(const ros::TimerEvent&) { publish(); }

  void publish()
  {
    footprint_.header.stamp = ros::Time::now();
    footprint_.header.frame_id = frame_id_;
    pub_.publish(footprint_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  std::string topic_;
  std::string frame_id_;
  double publish_rate_;
  geometry_msgs::PolygonStamped footprint_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_footprint_publisher");
  try
  {
    StaticFootprintPublisher publisher;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("static_footprint_publisher configuration error: %s", error.what());
    return 1;
  }
  return 0;
}
