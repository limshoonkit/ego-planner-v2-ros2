#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;

// Global variables for node functionality
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goals_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr new_goals_arrow_pub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_drones_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr user_goal_sub_;

rclcpp::Time last_publish_time_;
bool need_clear_ = false;

struct Selected_t
{
  int drone_id;
  Eigen::Vector3d p;
};
vector<Selected_t> drones_;

void displayArrowList(const vector<Eigen::Vector3d> &start, const vector<Eigen::Vector3d> &end, const double scale, const int id, const int32_t action)
{
  if (start.size() != end.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "start.size() != end.size(), return");
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  visualization_msgs::msg::Marker arrow;
  arrow.header.frame_id = "world";
  arrow.header.stamp = node_->get_clock()->now();
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = action;

  arrow.color.r = 0;
  arrow.color.g = 0;
  arrow.color.b = 0;
  arrow.color.a = 1.0;
  arrow.scale.x = scale;
  arrow.scale.y = 4 * scale;
  arrow.scale.z = 4 * scale;

  for (int i = 0; i < int(start.size()); i++)
  {
    geometry_msgs::msg::Point st, ed;
    st.x = start[i](0);
    st.y = start[i](1);
    st.z = start[i](2);
    ed.x = end[i](0);
    ed.y = end[i](1);
    ed.z = end[i](2);
    arrow.points.clear();
    arrow.points.push_back(st);
    arrow.points.push_back(ed);
    arrow.id = i + id;

    array.markers.push_back(arrow);
  }

  new_goals_arrow_pub_->publish(array);
}

void selected_drones_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  static rclcpp::Time last_select_time = rclcpp::Time(0);
  rclcpp::Time t_now = node_->get_clock()->now();
  if ((t_now - last_select_time).seconds() > 2.0)
  {
    drones_.clear();
  }
  Selected_t drone;
  drone.drone_id = atoi(msg->header.frame_id.substr(6, 10).c_str());
  drone.p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  drones_.push_back(drone);

  cout.precision(3);
  cout << "received drone " << drone.drone_id << " at " << drone.p.transpose() << ", total:" << drones_.size() << endl;

  last_select_time = t_now;
}

void user_goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < drones_.size(); ++i)
  {
    center += drones_[i].p;
  }
  center /= drones_.size();

  Eigen::Vector3d user_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d movment = user_goal - center;

  vector<Eigen::Vector3d> each_one_starts(drones_.size()), each_one_goals(drones_.size());
  for (size_t i = 0; i < drones_.size(); ++i)
  {
    each_one_starts[i] = drones_[i].p;
    each_one_goals[i] = drones_[i].p + movment;
    cout.precision(3);
    cout << "drone " << drones_[i].drone_id << ", start=" << drones_[i].p.transpose() << ", end=" << each_one_goals[i].transpose() << endl;
  }

  displayArrowList(each_one_starts, each_one_goals, 0.05, 0, visualization_msgs::msg::Marker::ADD);
  last_publish_time_ = node_->get_clock()->now();
  need_clear_ = true;

  for (size_t i = 0; i < drones_.size(); ++i)
  {
    quadrotor_msgs::msg::GoalSet goal_msg;
    goal_msg.drone_id = drones_[i].drone_id;
    goal_msg.goal[0] = each_one_goals[i](0);
    goal_msg.goal[1] = each_one_goals[i](1);
    goal_msg.goal[2] = each_one_goals[i](2);
    goals_pub_->publish(goal_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create node
  node_ = rclcpp::Node::make_shared("assign_goals");

  // Seed random number generator
  srand(floor(node_->get_clock()->now().seconds() * 10));

  // Create subscriptions
  selected_drones_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/rviz_selected_drones", 100, selected_drones_cb);
  
  user_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal", 10, user_goal_cb);

  // Create publishers
  goals_pub_ = node_->create_publisher<quadrotor_msgs::msg::GoalSet>("/goal_user2brig", 10);
  new_goals_arrow_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/new_goals_arrow", 10);

  // Initialize time
  last_publish_time_ = node_->get_clock()->now();

  RCLCPP_INFO(node_->get_logger(), "[assign_goals_node] Start running.");
  
  // Main loop
  rclcpp::Rate rate(100); // 100 Hz
  while (rclcpp::ok())
  {
    if (need_clear_ && (node_->get_clock()->now() - last_publish_time_).seconds() > 2.0)
    {
      need_clear_ = false;
      std::vector<Eigen::Vector3d> blank(1);
      blank[0] = Eigen::Vector3d::Zero();
      displayArrowList(blank, blank, 0.05, 0, visualization_msgs::msg::Marker::DELETEALL);
      cout << "DELETEALL Arrows." << endl;
    }

    rclcpp::spin_some(node_);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}