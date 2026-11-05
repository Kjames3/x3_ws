#include "nav2_mppi_dbas_controller/dbas_critic.hpp"
#include <cmath>

#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void DBaSCritic::initialize()
{
  auto node = parent_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to lock parent node");
    return;
  }
  
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(safe_radius_, "safe_radius", 0.6);
  getParam(collision_cost_, "collision_cost", 100000.0);
  // Drop estimates older than this (only when the message carries a stamp).
  getParam(max_msg_age_, "max_msg_age", 1.0);

  // QoS profile for pedestrian states
  rclcpp::QoS qos(10);
  ped_sub_ = node->create_subscription<x3_msgs::msg::PedestrianArray>(
    "/pedestrian_states", qos,
    std::bind(&DBaSCritic::pedestrianCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "DBaSCritic initialized with safe_radius: %.2f", safe_radius_);
}

void DBaSCritic::pedestrianCallback(const x3_msgs::msg::PedestrianArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(peds_mutex_);
  latest_peds_ = msg;
}

void DBaSCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  x3_msgs::msg::PedestrianArray::SharedPtr peds;
  {
    std::lock_guard<std::mutex> lock(peds_mutex_);
    peds = latest_peds_;
  }

  if (!peds || peds->pedestrians.empty()) {
    return; // No dynamic barriers
  }

  // /pedestrian_states is published in base_link; MPPI trajectories are in the
  // local costmap frame (odom). Without this transform people were placed as if
  // the robot sat at the odom origin facing +x.
  std::vector<Ped> ped_list;
  if (!pedsInCostmapFrame(*peds, ped_list) || ped_list.empty()) {
    return;
  }

  auto & trajectories_x = data.trajectories.x;
  auto & trajectories_y = data.trajectories.y;
  auto & costs = data.costs;

  size_t num_trajectories = trajectories_x.shape()[0];
  size_t trajectory_length = trajectories_x.shape()[1];
  double dt = data.model_dt;

  for (size_t i = 0; i < num_trajectories; ++i) {
    bool collision = false;
    for (size_t t = 0; t < trajectory_length && !collision; ++t) {
      double rx = trajectories_x(i, t);
      double ry = trajectories_y(i, t);
      double current_time = t * dt;

      for (const auto & ped : ped_list) {
        // Constant-velocity projection to time t
        double px = ped.x + ped.vx * current_time;
        double py = ped.y + ped.vy * current_time;
        if (std::hypot(rx - px, ry - py) < safe_radius_ + ped.r) {
          costs(i) += collision_cost_;
          collision = true;
          break;
        }
      }
    }
  }
}

bool DBaSCritic::pedsInCostmapFrame(
  const x3_msgs::msg::PedestrianArray & msg, std::vector<Ped> & out)
{
  auto node = parent_.lock();
  if (!node) {
    return false;
  }
  const std::string target = costmap_ros_->getGlobalFrameID();
  const std::string source = msg.header.frame_id.empty() ? "base_link" : msg.header.frame_id;
  const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
  const bool stamped = stamp.nanoseconds() != 0;

  if (stamped && max_msg_age_ > 0.0 &&
    (node->now() - stamp).seconds() > max_msg_age_)
  {
    return false;  // stale: the person has moved on, don't steer around a ghost
  }

  geometry_msgs::msg::TransformStamped tf;
  auto buffer = costmap_ros_->getTfBuffer();
  try {
    // Transform at the capture time if the buffer covers it, else the latest.
    tf2::TimePoint when = stamped ? tf2_ros::fromRclcpp(stamp) : tf2::TimePointZero;
    try {
      tf = buffer->lookupTransform(target, source, when);
    } catch (const tf2::ExtrapolationException &) {
      tf = buffer->lookupTransform(target, source, tf2::TimePointZero);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "DBaSCritic: no TF %s -> %s (%s); ignoring pedestrians", source.c_str(),
      target.c_str(), ex.what());
    return false;
  }

  const double yaw = tf2::getYaw(tf.transform.rotation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double tx = tf.transform.translation.x, ty = tf.transform.translation.y;

  out.clear();
  out.reserve(msg.pedestrians.size());
  for (const auto & p : msg.pedestrians) {
    Ped q;
    q.x = tx + c * p.position.x - s * p.position.y;
    q.y = ty + s * p.position.x + c * p.position.y;
    // Velocity is a ground velocity expressed in base axes: rotate only.
    q.vx = c * p.velocity.x - s * p.velocity.y;
    q.vy = s * p.velocity.x + c * p.velocity.y;
    q.r = p.radius;
    out.push_back(q);
  }
  return true;
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::DBaSCritic,
  mppi::critics::CriticFunction)
