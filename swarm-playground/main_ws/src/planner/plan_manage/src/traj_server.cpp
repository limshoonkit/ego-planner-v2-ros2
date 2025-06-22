#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class TrajServer : public rclcpp::Node
{
public:
    TrajServer() : Node("traj_server")
    {
        this->declare_parameter("traj_server/time_forward", -1.0);
        this->get_parameter("traj_server/time_forward", time_forward_);

        RCLCPP_INFO(this->get_logger(), "[Traj server]: time_forward: %f", time_forward_);

        // Publishers
        pos_cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
            "/position_cmd", 50);

        // Subscribers
        poly_traj_sub_ = this->create_subscription<traj_utils::msg::PolyTraj>(
            "planning/trajectory", 10,
            std::bind(&TrajServer::polyTrajCallback, this, std::placeholders::_1));
        
        heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "planning/heartbeat", 10,
            std::bind(&TrajServer::heartbeatCallback, this, std::placeholders::_1));

        // Timer
        cmd_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TrajServer::cmdCallback, this));

        // Parameters


        // Initialize variables
        last_yaw_ = 0.0;
        last_yawdot_ = 0.0;
        receive_traj_ = false;
        heartbeat_time_ = rclcpp::Time(0);
        last_pos_ = Eigen::Vector3d::Zero();

        // Sleep for initialization
        rclcpp::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(this->get_logger(), "[Traj server]: ready.");
    }

private:
    // ROS2 components
    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
    rclcpp::Subscription<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    // Trajectory variables
    bool receive_traj_;
    std::shared_ptr<poly_traj::Trajectory> traj_;
    double traj_duration_;
    rclcpp::Time start_time_;
    int traj_id_;
    rclcpp::Time heartbeat_time_;
    Eigen::Vector3d last_pos_;

    // Yaw control variables
    double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
    double time_forward_;

    // Configuration defines
    static constexpr bool FLIP_YAW_AT_END = false;
    static constexpr bool TURN_YAW_TO_CENTER_AT_END = false;

    void heartbeatCallback(const std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg; // Suppress unused parameter warning
        heartbeat_time_ = this->now();
    }

    void polyTrajCallback(const traj_utils::msg::PolyTraj::SharedPtr msg)
    {
        if (msg->order != 5)
        {
            RCLCPP_ERROR(this->get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
            return;
        }
        if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
        {
            RCLCPP_ERROR(this->get_logger(), "[traj_server] WRONG trajectory parameters");
            return;
        }

        int piece_nums = msg->duration.size();
        std::vector<double> dura(piece_nums);
        std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
        
        for (int i = 0; i < piece_nums; ++i)
        {
            int i6 = i * 6;
            cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
                msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
            cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
                msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
            cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
                msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

            dura[i] = msg->duration[i];
        }

        traj_ = std::make_shared<poly_traj::Trajectory>(dura, cMats);

        // Convert ROS2 time to rclcpp::Time
        start_time_ = msg->start_time;
        traj_duration_ = traj_->getTotalDuration();
        traj_id_ = msg->traj_id;

        receive_traj_ = true;
    }

    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
    {
        constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
        constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
        std::pair<double, double> yaw_yawdot(0, 0);
        
        Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                                  ? traj_->getPos(t_cur + time_forward_) - pos
                                  : traj_->getPos(traj_duration_) - pos;
        double yaw_temp = dir.norm() > 0.1
                              ? atan2(dir(1), dir(0))
                              : last_yaw_;

        double yawdot = 0;
        double d_yaw = yaw_temp - last_yaw_;
        if (d_yaw >= M_PI)
        {
            d_yaw -= 2 * M_PI;
        }
        if (d_yaw <= -M_PI)
        {
            d_yaw += 2 * M_PI;
        }

        const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
        const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
        double d_yaw_max;
        
        if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
        {
            d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
        }
        else
        {
            double t1 = (YDM - last_yawdot_) / YDDM;
            d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
        }

        if (fabs(d_yaw) > fabs(d_yaw_max))
        {
            d_yaw = d_yaw_max;
        }
        yawdot = d_yaw / dt;

        double yaw = last_yaw_ + d_yaw;
        if (yaw > M_PI)
            yaw -= 2 * M_PI;
        if (yaw < -M_PI)
            yaw += 2 * M_PI;
        
        yaw_yawdot.first = yaw;
        yaw_yawdot.second = yawdot;

        last_yaw_ = yaw_yawdot.first;
        last_yawdot_ = yaw_yawdot.second;

        yaw_yawdot.second = yaw_temp;

        return yaw_yawdot;
    }

    void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
    {
        quadrotor_msgs::msg::PositionCommand cmd;

        cmd.header.stamp = this->now();
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id_;

        cmd.position.x = p(0);
        cmd.position.y = p(1);
        cmd.position.z = p(2);
        cmd.velocity.x = v(0);
        cmd.velocity.y = v(1);
        cmd.velocity.z = v(2);
        cmd.acceleration.x = a(0);
        cmd.acceleration.y = a(1);
        cmd.acceleration.z = a(2);
        // cmd.jerk.x = j(0);
        // cmd.jerk.y = j(1);
        // cmd.jerk.z = j(2);
        cmd.yaw = y;
        cmd.yaw_dot = yd;
        
        pos_cmd_pub_->publish(cmd);

        last_pos_ = p;
    }

    void cmdCallback()
    {
        /* no publishing before receive traj_ and have heartbeat */
        if (heartbeat_time_.seconds() <= 1e-5)
        {
            return;
        }
        if (!receive_traj_)
            return;

        rclcpp::Time time_now = this->now();

        if ((time_now - heartbeat_time_).seconds() > 0.5)
        {
            RCLCPP_ERROR(this->get_logger(), "[traj_server] Lost heartbeat from the planner, is it dead?");

            receive_traj_ = false;
            publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
            return;
        }

        double t_cur = (time_now - start_time_).seconds();

        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), 
                       acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
        std::pair<double, double> yaw_yawdot(0, 0);

        static rclcpp::Time time_last = time_now;
        static bool finished = false;

        if (t_cur < traj_duration_ && t_cur >= 0.0)
        {
            pos = traj_->getPos(t_cur);
            vel = traj_->getVel(t_cur);
            acc = traj_->getAcc(t_cur);
            jer = traj_->getJer(t_cur);
            RCLCPP_INFO(this->get_logger(), "t_cur: %f", t_cur);
            RCLCPP_INFO(this->get_logger(), "pos: %f, %f, %f", pos(0), pos(1), pos(2));
            // RCLCPP_INFO(this->get_logger(), "vel: %f, %f, %f", vel(0), vel(1), vel(2));
            // RCLCPP_INFO(this->get_logger(), "acc: %f, %f, %f", acc(0), acc(1), acc(2));

            /*** calculate yaw ***/
            yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).seconds());
            /*** calculate yaw ***/

            time_last = time_now;
            last_yaw_ = yaw_yawdot.first;
            last_pos_ = pos;

            slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
            if (slowly_flip_yaw_target_ > M_PI)
                slowly_flip_yaw_target_ -= 2 * M_PI;
            if (slowly_flip_yaw_target_ < -M_PI)
                slowly_flip_yaw_target_ += 2 * M_PI;
            
            constexpr double CENTER[2] = {0.0, 0.0};
            slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

            // publish
            publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
            finished = false;
        }
        else if constexpr (FLIP_YAW_AT_END)
        {
            if (t_cur >= traj_duration_)
            {
                if (finished)
                    return;

                /* hover when finished traj_ */
                pos = traj_->getPos(traj_duration_);
                vel.setZero();
                acc.setZero();
                jer.setZero();

                if (slowly_flip_yaw_target_ > 0)
                {
                    last_yaw_ += (time_now - time_last).seconds() * M_PI / 2;
                    yaw_yawdot.second = M_PI / 2;
                    if (last_yaw_ >= slowly_flip_yaw_target_)
                    {
                        finished = true;
                    }
                }
                else
                {
                    last_yaw_ -= (time_now - time_last).seconds() * M_PI / 2;
                    yaw_yawdot.second = -M_PI / 2;
                    if (last_yaw_ <= slowly_flip_yaw_target_)
                    {
                        finished = true;
                    }
                }

                yaw_yawdot.first = last_yaw_;
                time_last = time_now;

                publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
            }
        }
        else if constexpr (TURN_YAW_TO_CENTER_AT_END)
        {
            if (t_cur >= traj_duration_)
            {
                if (finished)
                    return;

                /* hover when finished traj_ */
                pos = traj_->getPos(traj_duration_);
                vel.setZero();
                acc.setZero();
                jer.setZero();

                double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
                if (d_yaw >= M_PI)
                {
                    last_yaw_ += (time_now - time_last).seconds() * M_PI / 2;
                    yaw_yawdot.second = M_PI / 2;
                    if (last_yaw_ > M_PI)
                        last_yaw_ -= 2 * M_PI;
                }
                else if (d_yaw <= -M_PI)
                {
                    last_yaw_ -= (time_now - time_last).seconds() * M_PI / 2;
                    yaw_yawdot.second = -M_PI / 2;
                    if (last_yaw_ < -M_PI)
                        last_yaw_ += 2 * M_PI;
                }
                else if (d_yaw >= 0)
                {
                    last_yaw_ -= (time_now - time_last).seconds() * M_PI / 2;
                    yaw_yawdot.second = -M_PI / 2;
                    if (last_yaw_ <= slowly_turn_to_center_target_)
                        finished = true;
                }
                else
                {
                    last_yaw_ += (time_now - time_last).seconds() * M_PI / 2;
                    yaw_yawdot.second = M_PI / 2;
                    if (last_yaw_ >= slowly_turn_to_center_target_)
                        finished = true;
                }

                yaw_yawdot.first = last_yaw_;
                time_last = time_now;

                publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}