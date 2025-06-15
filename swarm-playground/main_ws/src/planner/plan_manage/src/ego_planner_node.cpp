#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("ego_planner_node");

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);

  rclcpp::spin(nh);

  rclcpp::shutdown();
  return 0;
}
