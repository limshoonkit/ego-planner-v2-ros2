#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rcutils/allocator.h>
#include <cstring>
#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_utils/msg/minco_traj.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

int udp_server_fd_, udp_send_fd_;
std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> other_odoms_sub_;
std::shared_ptr<rclcpp::Subscription<traj_utils::msg::MINCOTraj>> one_traj_sub_;
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> mandatory_stop_sub_;
std::shared_ptr<rclcpp::Subscription<quadrotor_msgs::msg::GoalSet>> goal_sub_;
std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub_;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr other_odoms_pub_;
rclcpp::Publisher<traj_utils::msg::MINCOTraj>::SharedPtr one_traj_pub_;
rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mandatory_stop_pub_;
rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goal_pub_;
rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

std::string udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
nav_msgs::msg::Odometry odom_msg_;
traj_utils::msg::MINCOTraj MINCOTraj_msg_;
std_msgs::msg::Empty stop_msg_;
quadrotor_msgs::msg::GoalSet goal_msg_;
sensor_msgs::msg::Joy joy_msg_;

enum MESSAGE_TYPE
{
  ODOM = 100,
  ONE_TRAJ,
  STOP,
  GOAL,
  JOY
} massage_type_;

int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("bridge_node"), "Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
  auto ptr = (uint8_t *)(udp_send_buf_);

  // Write message type
  *((MESSAGE_TYPE*)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  // Get type support for the message
  const rosidl_message_type_support_t * type_support = 
    rosidl_typesupport_cpp::get_message_type_support_handle<T>();

  // Initialize serialized message
  rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  
  // Allocate buffer for serialization (initial size, will grow if needed)
  size_t initial_capacity = 1024;
  rmw_ret_t ret = rmw_serialized_message_init(&serialized_msg, initial_capacity, &allocator);
  if (ret != RMW_RET_OK) {
    return -1; // Serialization failed
  }

  // Serialize the message
  ret = rmw_serialize(&msg, type_support, &serialized_msg);
  if (ret != RMW_RET_OK) {
    rmw_serialized_message_fini(&serialized_msg);
    return -1; // Serialization failed
  }

  uint32_t msg_size = serialized_msg.buffer_length;

  // Write message size
  *((uint32_t *)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  // Copy serialized data
  memcpy(ptr, serialized_msg.buffer, msg_size);

  // Cleanup
  rmw_serialized_message_fini(&serialized_msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T &msg)
{
  auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  // Read message size
  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  // Get type support for the message
  const rosidl_message_type_support_t * type_support = 
    rosidl_typesupport_cpp::get_message_type_support_handle<T>();

  // Initialize serialized message with received data
  rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  
  rmw_ret_t ret = rmw_serialized_message_init(&serialized_msg, msg_size, &allocator);
  if (ret != RMW_RET_OK) {
    return -1; // Failed to initialize
  }

  // Copy received data to serialized message buffer
  memcpy(serialized_msg.buffer, ptr, msg_size);
  serialized_msg.buffer_length = msg_size;

  // Deserialize the message
  ret = rmw_deserialize(&serialized_msg, type_support, &msg);
  
  // Cleanup
  rmw_serialized_message_fini(&serialized_msg);

  if (ret != RMW_RET_OK) {
    return -1; // Deserialization failed
  }

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}
rclcpp::Time last_odom_time;

void odom_sub_udp_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto node = rclcpp::Node::make_shared("bridge_node");
  auto now = node->now();
  if ((now - last_odom_time).seconds() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  last_odom_time = now;

  msg->child_frame_id = string("drone_") + std::to_string(drone_id_);

  int len = serializeTopic(MESSAGE_TYPE::ODOM, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "UDP SEND ERROR (1)!!!");
  }
}

void one_traj_sub_udp_cb(const traj_utils::msg::MINCOTraj::SharedPtr msg)
{
  auto node = rclcpp::Node::make_shared("bridge_node");
  int len = serializeTopic(MESSAGE_TYPE::ONE_TRAJ, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "UDP SEND ERROR (2)!!!");
  }
}

void mandatory_stop_sub_udp_cb(const std_msgs::msg::Empty::SharedPtr msg)
{
  auto node = rclcpp::Node::make_shared("bridge_node");
  int len = serializeTopic(MESSAGE_TYPE::STOP, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "UDP SEND ERROR (3)!!!");
  }
}

void goal_sub_udp_cb(const quadrotor_msgs::msg::GoalSet::SharedPtr msg)
{
  auto node = rclcpp::Node::make_shared("bridge_node");
  int len = serializeTopic(MESSAGE_TYPE::GOAL, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "UDP SEND ERROR (4)!!!");
  }
}

void joy_sub_udp_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto node = rclcpp::Node::make_shared("bridge_node");
  int len = serializeTopic(MESSAGE_TYPE::JOY, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node->get_logger(), "UDP SEND ERROR (5)!!!");
  }
}

void udp_recv_fun(std::shared_ptr<rclcpp::Node> node)
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len = sizeof(addr_client);

  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Socket receiver creation error!");
    exit(EXIT_FAILURE);
  }

  while (rclcpp::ok())
  {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, &addr_len)) < 0)
    {
      perror("recvfrom() < 0, error:");
      exit(EXIT_FAILURE);
    }

    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {
    case MESSAGE_TYPE::ODOM:
      if (valread == deserializeTopic(odom_msg_))
      {
        other_odoms_pub_->publish(odom_msg_);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Received message length not matches the sent one (2)!!!");
        continue;
      }
      break;
    case MESSAGE_TYPE::ONE_TRAJ:
      if (valread == deserializeTopic(MINCOTraj_msg_))
      {
        one_traj_pub_->publish(MINCOTraj_msg_);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Received message length not matches the sent one (2)!!!");
        continue;
      }
      break;
    case MESSAGE_TYPE::STOP:
      if (valread == deserializeTopic(stop_msg_))
      {
        mandatory_stop_pub_->publish(stop_msg_);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Received message length not matches the sent one (3)!!!");
        continue;
      }
      break;
    case MESSAGE_TYPE::GOAL:
      if (valread == deserializeTopic(goal_msg_))
      {
        goal_pub_->publish(goal_msg_);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Received message length not matches the sent one (4)!!!");
        continue;
      }
      break;
    case MESSAGE_TYPE::JOY:
      if (valread == deserializeTopic(joy_msg_))
      {
        joy_pub_->publish(joy_msg_);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Received message length not matches the sent one (5)!!!");
        continue;
      }
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown received message type???");
      break;
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("swarm_bridge");

  node->declare_parameter<std::string>("broadcast_ip", "127.0.0.255");
  node->declare_parameter<int>("drone_id", -1);
  node->declare_parameter<double>("odom_max_freq", 1000.0);

  node->get_parameter("broadcast_ip", udp_ip_);
  node->get_parameter("drone_id", drone_id_);
  node->get_parameter("odom_max_freq", odom_broadcast_freq_);

  if (drone_id_ == -1)
  {
    RCLCPP_WARN(node->get_logger(), "[swarm bridge] Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  other_odoms_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "my_odom", 10, odom_sub_udp_cb);

  other_odoms_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 10);

  one_traj_sub_ = node->create_subscription<traj_utils::msg::MINCOTraj>(
      "/broadcast_traj_from_planner", 100, one_traj_sub_udp_cb);
  one_traj_pub_ = node->create_publisher<traj_utils::msg::MINCOTraj>("/broadcast_traj_to_planner", 100);

  // mandatory_stop_sub_ = node->create_subscription<std_msgs::msg::Empty>(
  //     "/mandatory_stop_from_users", 100, mandatory_stop_sub_udp_cb);
  // mandatory_stop_pub_ = node->create_publisher<std_msgs::msg::Empty>("/mandatory_stop_to_planner", 100);

  goal_sub_ = node->create_subscription<quadrotor_msgs::msg::GoalSet>(
      "/goal_user2brig", 100, goal_sub_udp_cb);
  goal_pub_ = node->create_publisher<quadrotor_msgs::msg::GoalSet>("/goal_brig2plner", 100);

  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
      "/joystick_from_users", 100, joy_sub_udp_cb);
  joy_pub_ = node->create_publisher<sensor_msgs::msg::Joy>("/joystick_from_bridge", 100);

  std::thread udp_recv_thd(udp_recv_fun, node);
  udp_recv_thd.detach();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  RCLCPP_INFO(node->get_logger(), "[rosmsg_udp_bridge] start running");

  rclcpp::spin(node);

  close(udp_server_fd_);
  close(udp_send_fd_);

  rclcpp::shutdown();
  return 0;
}
