#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>

using std::placeholders::_1;

class GroundStationNode : public rclcpp::Node
{
public:
  GroundStationNode()
  : Node("manual_take_over_ground_station"), flag_mandatory_stoped_(false)
  {
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joystick_from_users", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&GroundStationNode::joy_sub_cb, this, _1));
  }

private:
  void joy_sub_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons.size() > 3 &&
        (msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3]))
    {
      flag_mandatory_stoped_ = true;
    }

    if (flag_mandatory_stoped_)
    {
      joy_pub_->publish(*msg);
    }
  }

  bool flag_mandatory_stoped_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundStationNode>());
  rclcpp::shutdown();
  return 0;
}
