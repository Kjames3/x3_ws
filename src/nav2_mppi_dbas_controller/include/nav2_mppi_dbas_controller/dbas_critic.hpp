#ifndef NAV2_MPPI_DBAS_CONTROLLER__DBAS_CRITIC_HPP_
#define NAV2_MPPI_DBAS_CONTROLLER__DBAS_CRITIC_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "x3_msgs/msg/pedestrian_array.hpp"

namespace mppi::critics
{

class DBaSCritic : public CriticFunction
{
public:
  DBaSCritic() = default;
  ~DBaSCritic() override = default;

  void initialize() override;
  void score(CriticData & data) override;

protected:
  void pedestrianCallback(const x3_msgs::msg::PedestrianArray::SharedPtr msg);

  rclcpp::Subscription<x3_msgs::msg::PedestrianArray>::SharedPtr ped_sub_;
  x3_msgs::msg::PedestrianArray::SharedPtr latest_peds_;
  std::mutex peds_mutex_;

  double safe_radius_;
  double collision_cost_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_DBAS_CONTROLLER__DBAS_CRITIC_HPP_
