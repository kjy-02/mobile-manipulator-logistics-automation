#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <scout_dwa/IntegrationHealth.h>
#include <geometry_msgs/PolygonStamped.h>
#include "footprint_validation.h"
#include "swept_footprint.h"

class CmdVelGuard
{
public:
  CmdVelGuard() : private_nh_("~"), have_raw_(false), have_odom_(false), have_obstacle_(false)
  {
    private_nh_.param<std::string>("raw_cmd_topic", raw_cmd_topic_, "/scout_dwa/cmd_vel_raw");
    private_nh_.param<std::string>("output_topic", output_topic_, "/scout_dwa/cmd_vel_safe");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/scout/odom");
    private_nh_.param<std::string>("obstacle_mode", obstacle_mode_, "map");
    private_nh_.param<std::string>("map_topic", map_topic_, "/scout_dwa/local_map");
    private_nh_.param<std::string>("scan_topic", scan_topic_, "/scout_dwa/scan");
    private_nh_.param<std::string>("global_frame", global_frame_, "map");
    private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    private_nh_.param("enabled", enabled_, false);
    private_nh_.param("publish_rate", publish_rate_, 20.0);
    private_nh_.param("raw_timeout", raw_timeout_, 0.30);
    private_nh_.param("odom_timeout", odom_timeout_, 0.30);
    private_nh_.param("obstacle_timeout", obstacle_timeout_, 0.30);
    private_nh_.param("tf_timeout", tf_timeout_, 0.30);
    private_nh_.param("max_linear_x", max_linear_x_, 0.50);
    private_nh_.param("max_angular_z", max_angular_z_, 0.7853);
    private_nh_.param("require_input_health", require_input_health_, true);
    private_nh_.param("require_sensor_health", require_sensor_health_, true);
    private_nh_.param("require_localization_health", require_localization_health_, true);
    private_nh_.param("health_timeout", health_timeout_, 0.30);
    private_nh_.param("max_future_offset", max_future_offset_, 0.05);
    private_nh_.param("rotation_clearance_radius", rotation_clearance_radius_, 0.0);
    private_nh_.param("require_config_health", require_config_health_, false);
    private_nh_.param("require_hardware_verification", require_hardware_verification_, false);
    private_nh_.param("hardware_verified", hardware_verified_, false);
    private_nh_.param("rotation_coverage_confirmed", rotation_coverage_confirmed_, false);
    private_nh_.param("require_rotation_coverage", require_rotation_coverage_, false);
    private_nh_.param("require_footprint", require_footprint_, false);
    private_nh_.param("use_swept_footprint", use_swept_footprint_, false);
    private_nh_.param("FOOTPRINT_PADDING", footprint_padding_, 0.0);
    private_nh_.param("footprint_padding", footprint_padding_, footprint_padding_);

    if (obstacle_mode_ != "map" && obstacle_mode_ != "scan")
      throw std::runtime_error("obstacle_mode must be 'map' or 'scan'");
    if (!(publish_rate_ > 0.0 && raw_timeout_ > 0.0 && odom_timeout_ > 0.0 &&
          obstacle_timeout_ > 0.0 && tf_timeout_ > 0.0 && max_linear_x_ > 0.0 &&
          max_angular_z_ > 0.0 && health_timeout_ > 0.0 && max_future_offset_ >= 0.0 &&
          rotation_clearance_radius_ >= 0.0))
      throw std::runtime_error("timeouts, limits, and publish_rate must be positive");
    for (double v : {publish_rate_, raw_timeout_, odom_timeout_, obstacle_timeout_,
                     tf_timeout_, max_linear_x_, max_angular_z_, health_timeout_,
                     max_future_offset_, rotation_clearance_radius_, footprint_padding_})
      if (!std::isfinite(v)) throw std::runtime_error("guard parameters must be finite");
    if (global_frame_.empty() || robot_frame_.empty())
      throw std::runtime_error("guard frames must not be empty");

    output_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 1);
    ready_pub_ = private_nh_.advertise<std_msgs::Bool>("ready", 1, true);
    reason_pub_ = private_nh_.advertise<std_msgs::String>("reason", 1, true);
    raw_sub_ = nh_.subscribe(raw_cmd_topic_, 1, &CmdVelGuard::rawCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &CmdVelGuard::odomCallback, this);
    if (obstacle_mode_ == "map")
      map_sub_ = nh_.subscribe(map_topic_, 1, &CmdVelGuard::mapCallback, this);
    else
      scan_sub_ = nh_.subscribe(scan_topic_, 1, &CmdVelGuard::scanCallback, this);
    input_health_sub_ = nh_.subscribe("/scout_dwa/input_health", 1, &CmdVelGuard::inputHealth, this);
    sensor_health_sub_ = nh_.subscribe("/scout_dwa/sensor_health", 1, &CmdVelGuard::sensorHealth, this);
    localization_health_sub_ = nh_.subscribe("/scout_dwa/localization_health", 1, &CmdVelGuard::localizationHealth, this);
    config_health_sub_ = nh_.subscribe("/scout_dwa/config_health", 1, &CmdVelGuard::configHealth, this);
    footprint_sub_ = nh_.subscribe("/scout_dwa/footprint", 1, &CmdVelGuard::footprintCallback, this);
    enable_service_ = private_nh_.advertiseService("set_enabled", &CmdVelGuard::setEnabled, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / publish_rate_), &CmdVelGuard::timerCallback, this);
  }

  ~CmdVelGuard()
  {
    output_pub_.publish(geometry_msgs::Twist());
  }

private:
  void rawCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    raw_cmd_ = *msg;
    raw_received_ = ros::Time::now();
    raw_wall_ = ros::WallTime::now();
    have_raw_ = true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    odom_received_ = ros::Time::now();
    odom_wall_ = ros::WallTime::now();
    odom_stamp_ = msg->header.stamp;
    const auto& t = msg->twist.twist;
    measured_cmd_ = t;
    odom_valid_ = std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
                  std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z) &&
                  !msg->header.frame_id.empty() && msg->child_frame_id == robot_frame_;
    have_odom_ = true;
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    obstacle_received_ = ros::Time::now();
    obstacle_wall_ = ros::WallTime::now();
    obstacle_stamp_ = msg->header.stamp;
    have_obstacle_ = msg->header.frame_id == robot_frame_ &&
        std::isfinite(msg->info.resolution) && msg->info.resolution > 0 &&
        msg->info.width > 0 && msg->info.height > 0 &&
        static_cast<uint64_t>(msg->info.width) * msg->info.height == msg->data.size() &&
        msg->data.size() <= 4000000 &&
        std::isfinite(msg->info.origin.position.x) && std::isfinite(msg->info.origin.position.y) &&
        std::fabs(msg->info.origin.orientation.w - 1.0) < 1e-6 &&
        std::fabs(msg->info.origin.orientation.x) < 1e-6 &&
        std::fabs(msg->info.origin.orientation.y) < 1e-6 &&
        std::fabs(msg->info.origin.orientation.z) < 1e-6;
    if (!have_obstacle_) return;
    occupied_cells_.clear();
    cell_radius_ = msg->info.resolution / std::sqrt(2.0);
    obstacle_min_distance_ = std::numeric_limits<double>::infinity();
    for (uint32_t y = 0; y < msg->info.height; ++y)
      for (uint32_t x = 0; x < msg->info.width; ++x)
        if (msg->data[x + y * msg->info.width] == 100)
        {
          geometry_msgs::Pose p;
          p.position.x=msg->info.origin.position.x+(x+.5)*msg->info.resolution;
          p.position.y=msg->info.origin.position.y+(y+.5)*msg->info.resolution;
          occupied_cells_.push_back(p);
          obstacle_min_distance_ = std::min(obstacle_min_distance_, std::hypot(msg->info.origin.position.x + (x + 0.5) * msg->info.resolution, msg->info.origin.position.y + (y + 0.5) * msg->info.resolution));
        }
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    obstacle_received_ = ros::Time::now();
    obstacle_wall_ = ros::WallTime::now();
    obstacle_stamp_ = msg->header.stamp;
    have_obstacle_ = msg->header.frame_id == robot_frame_ && !msg->ranges.empty() &&
        std::isfinite(msg->angle_increment) && msg->angle_increment > 0 &&
        std::isfinite(msg->range_min) && std::isfinite(msg->range_max) && msg->range_max > msg->range_min &&
        std::all_of(msg->ranges.begin(), msg->ranges.end(), [](float r) { return !std::isnan(r) && r >= 0; });
    if (!have_obstacle_) return;
    obstacle_min_distance_ = std::numeric_limits<double>::infinity();
    for (float range : msg->ranges) if (std::isfinite(range)) obstacle_min_distance_ = std::min(obstacle_min_distance_, static_cast<double>(range));
  }

  void setHealth(const scout_dwa::IntegrationHealthConstPtr& msg, bool& valid, ros::Time& stamp, ros::WallTime& wall)
  { valid = msg->valid; stamp = msg->header.stamp; wall = ros::WallTime::now(); }
  void inputHealth(const scout_dwa::IntegrationHealthConstPtr& m) { setHealth(m,input_health_valid_,input_health_stamp_,input_health_wall_); }
  void sensorHealth(const scout_dwa::IntegrationHealthConstPtr& m) { setHealth(m,sensor_health_valid_,sensor_health_stamp_,sensor_health_wall_); }
  void localizationHealth(const scout_dwa::IntegrationHealthConstPtr& m) { setHealth(m,localization_health_valid_,localization_health_stamp_,localization_health_wall_); }
  void configHealth(const scout_dwa::IntegrationHealthConstPtr& m) { setHealth(m,config_health_valid_,config_health_stamp_,config_health_wall_); }
  void footprintCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
  {
    double radius;
    footprint_valid_ = msg->header.frame_id == robot_frame_ &&
        validateFootprintPolygon(msg->polygon.points, radius) &&
        std::isfinite(footprint_padding_) && footprint_padding_ >= 0;
    if (footprint_valid_) rotation_clearance_radius_ = radius + std::sqrt(2.0) * footprint_padding_;
    if (footprint_valid_) footprint_points_ = msg->polygon.points;
  }
  bool setEnabled(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
  { enabled_ = request.data; if (!enabled_) output_pub_.publish(geometry_msgs::Twist()); response.success = true; response.message = enabled_ ? "enabled; all gates will be rechecked" : "disabled"; return true; }

  bool healthReady(bool required, bool valid, const ros::Time& stamp, const ros::WallTime& wall, const ros::Time& now) const
  { return !required || (valid && !stamp.isZero() && (now-stamp).toSec() <= health_timeout_ && (stamp-now).toSec() <= max_future_offset_ && (ros::WallTime::now()-wall).toSec() <= health_timeout_); }

  bool finiteAndBounded() const
  {
    return std::isfinite(raw_cmd_.linear.x) && std::isfinite(raw_cmd_.angular.z) &&
           std::fabs(raw_cmd_.linear.x) <= max_linear_x_ &&
           std::fabs(raw_cmd_.angular.z) <= max_angular_z_ &&
           std::fabs(raw_cmd_.linear.y) < 1e-9 && std::fabs(raw_cmd_.linear.z) < 1e-9 &&
           std::fabs(raw_cmd_.angular.x) < 1e-9 && std::fabs(raw_cmd_.angular.y) < 1e-9;
  }

  bool ready(const ros::Time& now, std::string& reason)
  {
    if (!enabled_) { reason="disabled"; return false; }
    if (require_hardware_verification_ && (!hardware_verified_ || ros::Time::isSimTime()))
    { reason="hardware verification pending or simulation time enabled"; return false; }
    if (require_footprint_ && !footprint_valid_)
    { reason="invalid or missing footprint"; return false; }
    if (!previous_now_.isZero() && now < previous_now_)
    { have_raw_=have_odom_=have_obstacle_=false; previous_now_=now; reason="ROS time moved backwards"; return false; }
    previous_now_=now;
    if (!have_raw_ || !have_odom_ || !have_obstacle_) { reason="waiting for raw, odom, or obstacle"; return false; }
    if (!healthReady(require_input_health_,input_health_valid_,input_health_stamp_,input_health_wall_,now) ||
        !healthReady(require_sensor_health_,sensor_health_valid_,sensor_health_stamp_,sensor_health_wall_,now) ||
        !healthReady(require_localization_health_,localization_health_valid_,localization_health_stamp_,localization_health_wall_,now) ||
        !healthReady(require_config_health_,config_health_valid_,config_health_stamp_,config_health_wall_,now))
    { reason="integration health gate rejected"; return false; }
    if (require_sensor_health_ && sensor_health_stamp_ != obstacle_stamp_)
    { reason="sensor health and obstacle stamps differ"; return false; }
    const ros::WallTime wall_now = ros::WallTime::now();
    if ((wall_now-raw_wall_).toSec()>raw_timeout_ || (wall_now-odom_wall_).toSec()>odom_timeout_ || (wall_now-obstacle_wall_).toSec()>obstacle_timeout_)
    { reason="input arrival timeout"; return false; }
    if (!odom_valid_) { reason="invalid odometry twist or frame"; return false; }
    if ((now - raw_received_).toSec() > raw_timeout_ ||
        (now - odom_received_).toSec() > odom_timeout_ ||
        (now - obstacle_received_).toSec() > obstacle_timeout_)
      { reason="ROS timestamp timeout"; return false; }
    if (obstacle_stamp_.isZero() || (now - obstacle_stamp_).toSec() > obstacle_timeout_ ||
        (obstacle_stamp_ - now).toSec() > max_future_offset_)
      { reason="stale obstacle stamp"; return false; }
    if (odom_stamp_.isZero() || (now - odom_stamp_).toSec() > odom_timeout_ ||
        (odom_stamp_ - now).toSec() > max_future_offset_)
      { reason="stale odometry stamp"; return false; }
    if (!finiteAndBounded())
      { reason="invalid command"; return false; }
    if (use_swept_footprint_ && require_footprint_)
    {
      if (obstacle_mode_ != "map") { reason="footprint sweep requires grid input"; return false; }
      const scout_collision::Bounds bounds(footprint_points_,footprint_padding_);
      // Operating assumption: 0.3 s reaction + measured speed / 1 m/s^2.
      // This is a software envelope, not a measured braking certification.
      const double horizon=.3+std::abs(measured_cmd_.linear.x);
      const double a=scout_collision::sweptClearance(occupied_cells_,bounds,
          raw_cmd_.linear.x,raw_cmd_.angular.z,horizon,cell_radius_);
      const double b=scout_collision::sweptClearance(occupied_cells_,bounds,
          measured_cmd_.linear.x,measured_cmd_.angular.z,horizon,cell_radius_);
      if ((a<=0 || b<=0) && (std::abs(raw_cmd_.linear.x)>1e-9 || std::abs(raw_cmd_.angular.z)>1e-9))
      { reason="predicted footprint path occupied"; return false; }
    }
    if (!use_swept_footprint_ && require_footprint_ && obstacle_min_distance_ <= rotation_clearance_radius_ &&
        (std::fabs(raw_cmd_.linear.x)>1e-9 || std::fabs(raw_cmd_.angular.z)>1e-9))
    { reason="current footprint circumscribed area occupied"; return false; }
    if (std::fabs(raw_cmd_.linear.x) < 1e-9 && std::fabs(raw_cmd_.angular.z) > 1e-9 &&
        require_rotation_coverage_ && (!rotation_coverage_confirmed_ || rotation_clearance_radius_ <= 0.0))
    { reason="rotation footprint or coverage unconfirmed"; return false; }
    if (!use_swept_footprint_ && std::fabs(raw_cmd_.linear.x) < 1e-9 && std::fabs(raw_cmd_.angular.z) > 1e-9 &&
        rotation_clearance_radius_ > 0.0 && obstacle_min_distance_ <= rotation_clearance_radius_)
    { reason="rotation clearance occupied"; return false; }
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(global_frame_, robot_frame_, ros::Time(0), transform);
      const bool ok = !transform.stamp_.isZero() && (now - transform.stamp_).toSec() <= tf_timeout_ && (transform.stamp_-now).toSec() <= max_future_offset_;
      reason = ok ? "ready" : "stale TF"; return ok;
    }
    catch (const tf::TransformException&)
    {
      reason="TF unavailable"; return false;
    }
  }

  void timerCallback(const ros::WallTimerEvent&)
  {
    std::string reason;
    const bool is_ready = ready(ros::Time::now(), reason);
    output_pub_.publish(is_ready ? raw_cmd_ : geometry_msgs::Twist());
    std_msgs::Bool ready_msg;
    ready_msg.data = is_ready;
    ready_pub_.publish(ready_msg);
    std_msgs::String reason_msg; reason_msg.data=reason; reason_pub_.publish(reason_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher output_pub_;
  ros::Publisher ready_pub_;
  ros::Publisher reason_pub_;
  ros::Subscriber raw_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber input_health_sub_, sensor_health_sub_, localization_health_sub_;
  ros::Subscriber config_health_sub_;
  ros::Subscriber footprint_sub_;
  ros::ServiceServer enable_service_;
  ros::WallTimer timer_;
  tf::TransformListener tf_listener_;

  std::string raw_cmd_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  std::string obstacle_mode_;
  std::string map_topic_;
  std::string scan_topic_;
  std::string global_frame_;
  std::string robot_frame_;
  bool enabled_;
  bool have_raw_;
  bool have_odom_;
  bool have_obstacle_;
  bool odom_valid_ = false, input_health_valid_ = false, sensor_health_valid_ = false, localization_health_valid_ = false;
  bool require_input_health_, require_sensor_health_, require_localization_health_;
  bool require_config_health_, require_hardware_verification_, hardware_verified_;
  bool rotation_coverage_confirmed_, require_rotation_coverage_;
  bool config_health_valid_ = false;
  bool require_footprint_, footprint_valid_ = false;
  bool use_swept_footprint_ = false;
  double cell_radius_ = 0;
  std::vector<geometry_msgs::Pose> occupied_cells_;
  std::vector<geometry_msgs::Point32> footprint_points_;
  double footprint_padding_;
  double publish_rate_;
  double raw_timeout_;
  double odom_timeout_;
  double obstacle_timeout_;
  double tf_timeout_;
  double max_linear_x_;
  double max_angular_z_;
  double health_timeout_, max_future_offset_, rotation_clearance_radius_;
  double obstacle_min_distance_ = std::numeric_limits<double>::infinity();
  ros::Time raw_received_;
  ros::Time odom_received_;
  ros::Time odom_stamp_;
  ros::Time obstacle_received_;
  ros::Time obstacle_stamp_;
  ros::Time input_health_stamp_, sensor_health_stamp_, localization_health_stamp_;
  ros::Time config_health_stamp_, previous_now_;
  ros::WallTime config_health_wall_;
  ros::WallTime raw_wall_, odom_wall_, obstacle_wall_, input_health_wall_, sensor_health_wall_, localization_health_wall_;
  geometry_msgs::Twist raw_cmd_;
  geometry_msgs::Twist measured_cmd_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_guard");
  try
  {
    CmdVelGuard guard;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("cmd_vel_guard configuration error: %s", error.what());
    return 1;
  }
  return 0;
}
