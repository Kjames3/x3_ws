#include "nav2_mppi_dbas_controller/dbas_critic.hpp"
#include <cmath>

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

  auto & trajectories_x = data.trajectories.x;
  auto & trajectories_y = data.trajectories.y;
  auto & costs = data.costs;

  size_t num_trajectories = trajectories_x.shape()[0];
  size_t trajectory_length = trajectories_x.shape()[1];
  double dt = data.model_dt;

  for (size_t i = 0; i < num_trajectories; ++i) {
    bool collision = false;
    for (size_t t = 0; t < trajectory_length; ++t) {
      if (collision) break;

      double rx = trajectories_x.at(i, t);
      double ry = trajectories_y.at(i, t);
      double current_time = t * dt;

      for (const auto & ped : peds->pedestrians) {
        // Project pedestrian position to future time t
        double px = ped.position.x + ped.velocity.x * current_time;
        double py = ped.position.y + ped.velocity.y * current_time;

        double dist = std::hypot(rx - px, ry - py);
        
        // Check barrier violation
        double combined_radius = safe_radius_ + ped.radius;
        if (dist < combined_radius) {
          costs[i] += collision_cost_;
          collision = true;
          break;
        }
      }
    }
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::DBaSCritic,
  mppi::critics::CriticFunction)
