#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;
using std::placeholders::_1;

namespace ego_planner
{

  class EGOReplanFSM : public rclcpp::Node
  {
  public:
    EGOReplanFSM();
    ~EGOReplanFSM() {}

    void init(rclcpp::Node::SharedPtr node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    rclcpp::Node::SharedPtr node_;
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                             // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    std::vector<Eigen::Vector3d> wps_;

    /* ROS2 utils */
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<quadrotor_msgs::msg::GoalSet>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
    rclcpp::Subscription<traj_utils::msg::MINCOTraj>::SharedPtr broadcast_ploytraj_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mandatory_stop_sub_;
    rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<traj_utils::msg::MINCOTraj>::SharedPtr broadcast_ploytraj_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_height_pub_;

    /* state machine functions */
    void execFSMCallback();
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback();
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const quadrotor_msgs::msg::GoalSet::SharedPtr msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp);
    bool mondifyInCollisionFinalGoal();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::msg::Empty::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void triggerCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::msg::MINCOTraj::SharedPtr msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);
  };

} // namespace ego_planner

#endif