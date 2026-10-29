#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <scout_dwa/IntegrationHealth.h>

class CloudObstacleAdapter
{
public:
  CloudObstacleAdapter() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/velodyne_points");
    private_nh_.param<std::string>("map_topic", map_topic_, "/scout_dwa/local_map");
    private_nh_.param<std::string>("safety_map_topic", safety_map_topic_, "/scout_dwa/safety_map");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/scout_dwa/scan");
    private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
    private_nh_.param("publish_map", publish_map_, true);
    private_nh_.param("publish_scan", publish_scan_, true);
    private_nh_.param("map_size_x", map_size_x_, 6.0);
    private_nh_.param("map_size_y", map_size_y_, 6.0);
    private_nh_.param("map_resolution", map_resolution_, 0.05);
    private_nh_.param("min_z", min_z_, -0.30);
    private_nh_.param("max_z", max_z_, 0.80);
    private_nh_.param("min_range", min_range_, 0.10);
    private_nh_.param("max_range", max_range_, 5.00);
    private_nh_.param("self_filter_enabled", self_filter_enabled_, false);
    private_nh_.param("self_min_x", self_min_x_, -0.50);
    private_nh_.param("self_max_x", self_max_x_, 0.50);
    private_nh_.param("self_min_y", self_min_y_, -0.40);
    private_nh_.param("self_max_y", self_max_y_, 0.40);
    private_nh_.param("scan_angle_min", scan_angle_min_, -M_PI);
    private_nh_.param("scan_angle_max", scan_angle_max_, M_PI);
    private_nh_.param("scan_angle_increment", scan_angle_increment_, M_PI / 180.0);
    private_nh_.param("max_cloud_age", max_cloud_age_, 0.30);
    private_nh_.param("max_future_offset", max_future_offset_, 0.05);
    private_nh_.param("max_points", max_points_, 2000000);
    private_nh_.param("allow_empty_cloud", allow_empty_cloud_, false);
    private_nh_.param("coverage_confirmed", coverage_confirmed_, false);
    private_nh_.param("planner_front_only", planner_front_only_, false);
    private_nh_.param("planner_half_angle_deg", planner_half_angle_deg_, 60.0);

    validateParameters();

    if (publish_map_)
      map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1);
    safety_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(safety_map_topic_, 1);
    if (publish_scan_)
      scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 1);
    health_pub_ = nh_.advertise<scout_dwa::IntegrationHealth>("/scout_dwa/sensor_health", 1);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/scout_dwa/sensor_diagnostics", 1);
    cloud_sub_ = nh_.subscribe(input_topic_, 1, &CloudObstacleAdapter::cloudCallback, this);
  }

private:
  void validateParameters()
  {
    if (!std::isfinite(planner_half_angle_deg_) || planner_half_angle_deg_ <= 0.0 || planner_half_angle_deg_ > 180.0 ||
        safety_map_topic_.empty() || safety_map_topic_ == map_topic_)
      throw std::runtime_error("invalid planner angular filter or safety map topic");
    if (!publish_map_ && !publish_scan_)
      throw std::runtime_error("publish_map and publish_scan cannot both be false");
    if (target_frame_.empty())
      throw std::runtime_error("target_frame cannot be empty");
    if (!(map_size_x_ > 0.0 && map_size_y_ > 0.0 && map_resolution_ > 0.0))
      throw std::runtime_error("map dimensions and resolution must be positive");
    if (!(min_z_ < max_z_ && 0.0 <= min_range_ && min_range_ < max_range_))
      throw std::runtime_error("invalid height or range filter");
    if (self_filter_enabled_ &&
        !(self_min_x_ < self_max_x_ && self_min_y_ < self_max_y_))
      throw std::runtime_error("invalid self filter bounds");
    if (!(scan_angle_min_ < scan_angle_max_ && scan_angle_increment_ > 0.0))
      throw std::runtime_error("invalid scan angular configuration");
    if (!(max_cloud_age_ > 0.0 && max_future_offset_ >= 0.0 && max_points_ > 0))
      throw std::runtime_error("invalid cloud timing or size limits");
    for (double v : {map_size_x_, map_size_y_, map_resolution_, min_z_, max_z_,
                     min_range_, max_range_, scan_angle_min_, scan_angle_max_,
                     scan_angle_increment_, max_cloud_age_, max_future_offset_})
      if (!std::isfinite(v)) throw std::runtime_error("adapter parameters must be finite");
    if (std::ceil(map_size_x_ / map_resolution_) * std::ceil(map_size_y_ / map_resolution_) > 4000000 ||
        (scan_angle_max_ - scan_angle_min_) / scan_angle_increment_ > 1000000)
      throw std::runtime_error("adapter resource limit exceeded");
    if (allow_empty_cloud_ && !coverage_confirmed_)
      throw std::runtime_error("allow_empty_cloud requires explicit coverage_confirmed");
  }

  void publishHealth(const ros::Time& stamp, bool valid, const std::string& reason)
  {
    scout_dwa::IntegrationHealth health;
    health.header.stamp = stamp;
    health.header.frame_id = target_frame_;
    health.valid = valid;
    health.reason = reason;
    health_pub_.publish(health);
  }

  bool validLayout(const sensor_msgs::PointCloud2& msg) const
  {
    int x = 0, y = 0, z = 0;
    uint32_t ox = 0, oy = 0, oz = 0;
    for (const auto& field : msg.fields)
    {
      const bool coordinate = field.name == "x" || field.name == "y" || field.name == "z";
      if (coordinate && (field.datatype != sensor_msgs::PointField::FLOAT32 || field.count != 1 ||
          static_cast<uint64_t>(field.offset) + 4 > msg.point_step))
        return false;
      if (field.name == "x") { ++x; ox=field.offset; }
      if (field.name == "y") { ++y; oy=field.offset; }
      if (field.name == "z") { ++z; oz=field.offset; }
    }
    const uint64_t points = static_cast<uint64_t>(msg.width) * msg.height;
    return x == 1 && y == 1 && z == 1 && !msg.is_bigendian && msg.height > 0 &&
           std::abs(int64_t(ox)-oy) >= 4 && std::abs(int64_t(ox)-oz) >= 4 && std::abs(int64_t(oy)-oz) >= 4 &&
           msg.point_step >= 12 && msg.point_step <= 1024 &&
           msg.row_step >= static_cast<uint64_t>(msg.point_step) * msg.width &&
           msg.data.size() == static_cast<uint64_t>(msg.row_step) * msg.height &&
           msg.data.size() <= 128ULL * 1024 * 1024 && points <= static_cast<uint64_t>(max_points_);
  }

  bool pointAccepted(const pcl::PointXYZ& point) const
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      return false;
    const double range = std::hypot(point.x, point.y);
    if (point.z < min_z_ || point.z > max_z_ || range < min_range_ || range > max_range_)
      return false;
    if (self_filter_enabled_ && point.x >= self_min_x_ && point.x <= self_max_x_ &&
        point.y >= self_min_y_ && point.y <= self_max_y_)
      return false;
    return true;
  }

  nav_msgs::OccupancyGrid makeMap(const std_msgs::Header& header) const
  {
    nav_msgs::OccupancyGrid map;
    map.header = header;
    map.header.frame_id = target_frame_;
    map.info.map_load_time = header.stamp;
    map.info.resolution = map_resolution_;
    map.info.width = static_cast<uint32_t>(std::ceil(map_size_x_ / map_resolution_));
    map.info.height = static_cast<uint32_t>(std::ceil(map_size_y_ / map_resolution_));
    map.info.origin.position.x = -0.5 * map.info.width * map.info.resolution;
    map.info.origin.position.y = -0.5 * map.info.height * map.info.resolution;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<size_t>(map.info.width) * map.info.height, 0);
    return map;
  }

  sensor_msgs::LaserScan makeScan(const std_msgs::Header& header) const
  {
    sensor_msgs::LaserScan scan;
    scan.header = header;
    scan.header.frame_id = target_frame_;
    scan.angle_min = scan_angle_min_;
    scan.angle_max = scan_angle_max_;
    scan.angle_increment = scan_angle_increment_;
    scan.range_min = min_range_;
    scan.range_max = max_range_;
    const size_t count = static_cast<size_t>(std::floor(
        (scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1U;
    scan.ranges.assign(count, std::numeric_limits<float>::infinity());
    return scan;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    const ros::Time now = ros::Time::now();
    if (msg->header.stamp.isZero() || (now - msg->header.stamp).toSec() > max_cloud_age_ ||
        (msg->header.stamp - now).toSec() > max_future_offset_ || !validLayout(*msg))
    {
      publishHealth(msg->header.stamp, false, "invalid cloud timestamp or layout");
      return;
    }
    sensor_msgs::PointCloud2 transformed_msg;
    if (!pcl_ros::transformPointCloud(target_frame_, *msg, transformed_msg, tf_listener_))
    {
      ROS_WARN_THROTTLE(1.0, "Point cloud transform to %s failed", target_frame_.c_str());
      publishHealth(msg->header.stamp, false, "point cloud TF unavailable");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    try
    {
      pcl::fromROSMsg(transformed_msg, cloud);
    }
    catch (const std::exception& error)
    {
      ROS_WARN_THROTTLE(1.0, "Point cloud conversion failed: %s", error.what());
      publishHealth(msg->header.stamp, false, "point cloud conversion failed");
      return;
    }

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid safety_map = makeMap(transformed_msg.header);
    sensor_msgs::LaserScan scan;
    if (publish_map_)
      map = makeMap(transformed_msg.header);
    if (publish_scan_)
      scan = makeScan(transformed_msg.header);

    size_t accepted = 0, nonfinite = 0, filtered = 0, height_removed = 0, range_removed = 0, self_removed = 0;
    size_t planner_angle_removed = 0;
    for (const auto& point : cloud.points)
    {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      { ++nonfinite; continue; }
      if (!pointAccepted(point))
      { ++filtered;
        if (point.z < min_z_ || point.z > max_z_) ++height_removed;
        else if (std::hypot(point.x, point.y) < min_range_ || std::hypot(point.x, point.y) > max_range_) ++range_removed;
        else ++self_removed;
        continue;
      }
      ++accepted;
      const bool in_planner_sector = !planner_front_only_ ||
          std::fabs(std::atan2(point.y, point.x)) <= planner_half_angle_deg_ * M_PI / 180.0;
      if (!in_planner_sector) ++planner_angle_removed;

      {
        const int x = static_cast<int>(std::floor(
            (point.x - safety_map.info.origin.position.x) / safety_map.info.resolution));
        const int y = static_cast<int>(std::floor(
            (point.y - safety_map.info.origin.position.y) / safety_map.info.resolution));
        if (x >= 0 && y >= 0 && x < static_cast<int>(safety_map.info.width) &&
            y < static_cast<int>(safety_map.info.height))
        {
          const size_t index = static_cast<size_t>(x) + static_cast<size_t>(y) * safety_map.info.width;
          safety_map.data[index] = 100;
          if (publish_map_ && in_planner_sector) map.data[index] = 100;
        }
      }

      if (publish_scan_ && in_planner_sector)
      {
        const double angle = std::atan2(point.y, point.x);
        if (angle < scan.angle_min || angle > scan.angle_max)
          continue;
        const size_t index = static_cast<size_t>(std::floor(
            (angle - scan.angle_min) / scan.angle_increment));
        if (index < scan.ranges.size())
        {
          const float range = std::hypot(point.x, point.y);
          scan.ranges[index] = std::min(scan.ranges[index], range);
        }
      }
    }

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header = msg->header;
    diagnostic_msgs::DiagnosticStatus diagnostic;
    diagnostic.name = "scout_dwa/cloud_filter";
    diagnostic.level = accepted ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    diagnostic.message = "Point counts; coverage is separately confirmed";
    for (const auto& count : std::vector<std::pair<std::string, size_t>>{
        {"input", cloud.size()}, {"finite", cloud.size()-nonfinite}, {"nonfinite", nonfinite},
        {"height_removed", height_removed}, {"range_removed", range_removed}, {"self_removed", self_removed},
        {"filtered", filtered}, {"accepted", accepted}, {"planner_angle_removed", planner_angle_removed}})
    {
      diagnostic_msgs::KeyValue value; value.key = count.first; value.value = std::to_string(count.second);
      diagnostic.values.push_back(value);
    }
    diagnostics.status.push_back(diagnostic);
    diagnostics_pub_.publish(diagnostics);
    if (accepted == 0 && (!allow_empty_cloud_ || nonfinite > 0 || filtered > 0))
    {
      publishHealth(msg->header.stamp, false, "cloud contains no accepted obstacle observations");
      return;
    }
    if (publish_map_)
      map_pub_.publish(map);
    safety_map_pub_.publish(safety_map);
    if (publish_scan_)
      scan_pub_.publish(scan);
    publishHealth(msg->header.stamp, coverage_confirmed_,
                  coverage_confirmed_ ? "sensor input and coverage valid" : "coverage unconfirmed; calculation only");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher map_pub_;
  ros::Publisher safety_map_pub_;
  ros::Publisher scan_pub_;
  ros::Publisher health_pub_;
  ros::Publisher diagnostics_pub_;
  tf::TransformListener tf_listener_;

  std::string input_topic_;
  std::string map_topic_;
  std::string safety_map_topic_;
  std::string scan_topic_;
  std::string target_frame_;
  bool publish_map_;
  bool publish_scan_;
  bool self_filter_enabled_;
  double map_size_x_;
  double map_size_y_;
  double map_resolution_;
  double min_z_;
  double max_z_;
  double min_range_;
  double max_range_;
  double self_min_x_;
  double self_max_x_;
  double self_min_y_;
  double self_max_y_;
  double scan_angle_min_;
  double scan_angle_max_;
  double scan_angle_increment_;
  double max_cloud_age_;
  double max_future_offset_;
  int max_points_;
  bool allow_empty_cloud_;
  bool coverage_confirmed_;
  bool planner_front_only_;
  double planner_half_angle_deg_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_obstacle_adapter");
  try
  {
    CloudObstacleAdapter adapter;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("cloud_obstacle_adapter configuration error: %s", error.what());
    return 1;
  }
  return 0;
}
