#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <traj_utils/msg/minco_traj.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unistd.h>
#include "reliable_bridge.hpp"

using namespace std;

class SwarmBridgeNode : public rclcpp::Node
{
public:
  SwarmBridgeNode() : Node("swarm_bridge")
  {
    // Declare and get parameters
    this->declare_parameter<int>("self_id", -1);
    this->declare_parameter<bool>("is_ground_station", false);
    this->declare_parameter<int>("drone_num", 0);
    this->declare_parameter<int>("ground_station_num", 0);
    this->declare_parameter<double>("odom_max_freq", 1000.0);

    this->get_parameter("self_id", self_id_);
    this->get_parameter("is_ground_station", is_groundstation_);
    this->get_parameter("drone_num", drone_num_);
    this->get_parameter("ground_station_num", ground_station_num_);
    this->get_parameter("odom_max_freq", odom_broadcast_freq_);

    id_list_.resize(drone_num_ + ground_station_num_);
    ip_list_.resize(drone_num_ + ground_station_num_);
    for (int i = 0; i < drone_num_ + ground_station_num_; ++i)
    {
      string param_name = (i < drone_num_ ? "drone_ip_" + to_string(i) : "ground_station_ip_" + to_string(i-drone_num_));
      this->declare_parameter<string>(param_name, "127.0.0.1");
      this->get_parameter(param_name, ip_list_[i]);
      id_list_[i] = i;
    }
    self_id_in_bridge_ = self_id_;
    if (is_groundstation_)
    {
      self_id_in_bridge_ = remap_ground_station_id(self_id_);
    }
    if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99)
    {
      RCLCPP_WARN(this->get_logger(), "[swarm bridge] Wrong self_id!");
      exit(EXIT_FAILURE);
    }

    bridge.reset(new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));

    // Publishers and Subscribers
    other_odoms_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 10);
    other_odoms_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "my_odom", 10, std::bind(&SwarmBridgeNode::odom_sub_cb, this, std::placeholders::_1));

    object_odoms_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/object_odom_brig2plner", 10);
    object_odoms_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/object_odom_dtc2brig", 10, std::bind(&SwarmBridgeNode::object_odom_sub_udp_cb, this, std::placeholders::_1));

    one_traj_pub_ = this->create_publisher<traj_utils::msg::MINCOTraj>("/broadcast_traj_to_planner", 100);
    one_traj_sub_ = this->create_subscription<traj_utils::msg::MINCOTraj>(
      "/broadcast_traj_from_planner", 100, std::bind(&SwarmBridgeNode::one_traj_sub_cb, this, std::placeholders::_1));

    joystick_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joystick_from_bridge", 100);
    joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joystick_from_users", 100, std::bind(&SwarmBridgeNode::joystick_sub_cb, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<quadrotor_msgs::msg::GoalSet>("/goal_brig2plner", 100);
    goal_sub_ = this->create_subscription<quadrotor_msgs::msg::GoalSet>(
      "/goal_user2brig", 100, std::bind(&SwarmBridgeNode::goal_sub_cb, this, std::placeholders::_1));

    // Register bridge callbacks
    register_callbak_to_all_drones("/odom", std::bind(&SwarmBridgeNode::odom_bridge_cb, this, std::placeholders::_1, std::placeholders::_2));
    register_callbak_to_all_drones("/object_odom", std::bind(&SwarmBridgeNode::object_odom_bridge_cb, this, std::placeholders::_1, std::placeholders::_2));
    bridge->register_callback_for_all("/traj_from_planner", std::bind(&SwarmBridgeNode::traj_bridge_cb, this, std::placeholders::_1, std::placeholders::_2));
    register_callbak_to_all_groundstation("/joystick", std::bind(&SwarmBridgeNode::joystick_bridge_cb, this, std::placeholders::_1, std::placeholders::_2));
    register_callbak_to_all_groundstation("/goal", std::bind(&SwarmBridgeNode::goal_bridge_cb, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  std::vector<int> id_list_;
  std::vector<string> ip_list_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr other_odoms_sub_, object_odoms_sub_;
  rclcpp::Subscription<traj_utils::msg::MINCOTraj>::SharedPtr one_traj_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::GoalSet>::SharedPtr goal_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr other_odoms_pub_, object_odoms_pub_;
  rclcpp::Publisher<traj_utils::msg::MINCOTraj>::SharedPtr one_traj_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystick_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goal_pub_;

  int self_id_;
  int self_id_in_bridge_;
  int drone_num_;
  int ground_station_num_;
  double odom_broadcast_freq_;
  bool is_groundstation_;

  unique_ptr<ReliableBridge> bridge;

  rclcpp::Time t_last_odom_{0, 0, RCL_ROS_TIME};

  inline int remap_ground_station_id(int id)
  {
    return id + drone_num_;
  }

  template <typename T>
  int send_to_all_drone_except_me(string topic, T &msg)
  {
    int err_code = 0;
    for (int i = 0; i < drone_num_; ++i)
    {
      if (i == self_id_in_bridge_)
        continue;
      err_code += bridge->send_msg_to_one(i, topic, msg);
      if (err_code < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "[Bridge] SEND ERROR %s !!", typeid(T).name());
      }
    }
    return err_code;
  }

  template <typename T>
  int send_to_all_groundstation_except_me(string topic, T &msg)
  {
    int err_code = 0;
    for (int i = 0; i < ground_station_num_; ++i)
    {
      int ind = remap_ground_station_id(i);
      if (ind == self_id_in_bridge_)
        continue;
      err_code += bridge->send_msg_to_one(ind, topic, msg);
      if (err_code < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "[Bridge] SEND ERROR %s !!", typeid(T).name());
      }
    }
    return err_code;
  }

  void register_callbak_to_all_groundstation(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
  {
    for (int i = 0; i < ground_station_num_; ++i)
    {
      int ind = remap_ground_station_id(i);
      if (ind == self_id_in_bridge_)
        continue;
      bridge->register_callback(ind, topic_name, callback);
    }
  }

  void register_callbak_to_all_drones(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
  {
    for (int i = 0; i < drone_num_; ++i)
    {
      if (i == self_id_in_bridge_)
        continue;
      bridge->register_callback(i, topic_name, callback);
    }
  }

  // Callbacks
  void odom_sub_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto t_now = this->now();
    if ((t_now - t_last_odom_).seconds() * odom_broadcast_freq_ < 1.0)
      return;
    t_last_odom_ = t_now;

    msg->child_frame_id = string("drone_") + std::to_string(self_id_);
    other_odoms_pub_->publish(*msg);
    send_to_all_drone_except_me("/odom", *msg);
    send_to_all_groundstation_except_me("/odom", *msg);
  }

  void object_odom_sub_udp_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    msg->child_frame_id = string("obj_") + std::to_string(self_id_);
    object_odoms_pub_->publish(*msg);
    send_to_all_drone_except_me("/object_odom", *msg);
    send_to_all_groundstation_except_me("/object_odom", *msg);
  }

  void one_traj_sub_cb(const traj_utils::msg::MINCOTraj::SharedPtr msg)
  {
    one_traj_pub_->publish(*msg);
    if (bridge->send_msg_to_all("/traj_from_planner", *msg))
    {
      RCLCPP_ERROR(this->get_logger(), "[Bridge] SEND ERROR (ONE_TRAJ)!!!");
    }
  }

  void joystick_sub_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joystick_pub_->publish(*msg);
    send_to_all_drone_except_me("/joystick", *msg);
  }

  void goal_sub_cb(const quadrotor_msgs::msg::GoalSet::SharedPtr msg)
  {
    if (msg->drone_id == self_id_in_bridge_)
    {
      goal_pub_->publish(*msg);
      return;
    }
    if (bridge->send_msg_to_one(msg->drone_id, "/goal", *msg) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "[Bridge] SEND ERROR (GOAL)!!!");
    }
  }

  // Bridge callbacks
  void odom_bridge_cb(int ID, ros::SerializedMessage &m)
  {
    nav_msgs::msg::Odometry odom_msg_;
    ros::serialization::deserializeMessage(m, odom_msg_);
    other_odoms_pub_->publish(odom_msg_);
  }

  void object_odom_bridge_cb(int ID, ros::SerializedMessage &m)
  {
    nav_msgs::msg::Odometry object_odom_msg_;
    ros::serialization::deserializeMessage(m, object_odom_msg_);
    object_odoms_pub_->publish(object_odom_msg_);
  }

  void goal_bridge_cb(int ID, ros::SerializedMessage &m)
  {
    quadrotor_msgs::msg::GoalSet goal_msg_;
    ros::serialization::deserializeMessage(m, goal_msg_);
    goal_pub_->publish(goal_msg_);
  }

  void traj_bridge_cb(int ID, ros::SerializedMessage &m)
  {
    traj_utils::msg::MINCOTraj MINCOTraj_msg_;
    ros::serialization::deserializeMessage(m, MINCOTraj_msg_);
    one_traj_pub_->publish(MINCOTraj_msg_);
  }

  void joystick_bridge_cb(int ID, ros::SerializedMessage &m)
  {
    sensor_msgs::msg::Joy joystick_msg_;
    ros::serialization::deserializeMessage(m, joystick_msg_);
    joystick_pub_->publish(joystick_msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SwarmBridgeNode>();
  rclcpp::spin(node);
  node->bridge->StopThread();
  rclcpp::shutdown();
  return 0;
}
