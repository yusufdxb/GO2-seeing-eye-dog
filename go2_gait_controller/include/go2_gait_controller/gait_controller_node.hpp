#pragma once
// ============================================================
// go2_gait_controller/gait_controller_node.hpp
// ============================================================
// ROS 2 Lifecycle Node — GO2 Gait State Machine
//
// Gait states:
//   IDLE   → robot is powered but not moving
//   STAND  → robot is standing, joints locked at nominal pose
//   WALK   → slow walking gait (low frequency, high stability)
//   TROT   → trotting gait (diagonal pairs, higher speed)
//
// Transitions are triggered via /go2/gait_command (std_msgs/String)
// Joint trajectories are published to /joint_group_effort_controller/joint_trajectory
//
// Author: Yusuf Guenena
// ============================================================

#ifndef GO2_GAIT_CONTROLLER__GAIT_CONTROLLER_NODE_HPP_
#define GO2_GAIT_CONTROLLER__GAIT_CONTROLLER_NODE_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace go2_gait_controller
{

// ── Gait state enum ─────────────────────────────────────────
enum class GaitState
{
  IDLE,
  STAND,
  WALK,
  TROT
};

// ── Joint name order (matches CHAMP-style GO2 URDF) ─────────
// Leg order: FR, FL, RR, RL (internal gait indexing)
// URDF mapping: FR=rf, FL=lf, RR=rh, RL=lh
static const std::vector<std::string> JOINT_NAMES = {
  "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",   // FR
  "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",   // FL
  "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint",   // RR
  "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"    // RL
};

// ── Nominal standing pose (radians) ─────────────────────────
// [hip, upper_leg, lower_leg] per leg — FR(rf), FL(lf), RR(rh), RL(lh)
static const std::vector<double> STAND_POSE = {
   0.0,  0.9, -1.8,   // FR
   0.0,  0.9, -1.8,   // FL
   0.0,  0.9, -1.8,   // RR
   0.0,  0.9, -1.8    // RL
};

// ── Gait parameters ──────────────────────────────────────────
struct GaitParams
{
  double frequency;        // Hz — gait cycle frequency
  double stride_height;    // m  — foot lift height
  double stride_length;    // m  — forward step length
  double body_height;      // m  — nominal body height above ground
  double stance_ratio;     // [0,1] — fraction of cycle in stance
};

static const std::map<GaitState, GaitParams> GAIT_PARAMS = {
  {GaitState::WALK, {1.5, 0.05, 0.08, 0.27, 0.65}},
  {GaitState::TROT, {2.5, 0.06, 0.12, 0.27, 0.55}},
};

// ── Leg index mapping ────────────────────────────────────────
// Trot: diagonal pairs (FR+RL) and (FL+RR) alternate
static const std::vector<std::vector<int>> TROT_PAIRS = {{0, 3}, {1, 2}};

// ── Controller node ──────────────────────────────────────────
class GaitControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GaitControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GaitControllerNode() = default;

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // ── Control loop ────────────────────────────────────────
  void control_loop();

  // ── Gait generation (returns 12 joint positions) ────────
  std::vector<double> generate_stand_positions();
  std::vector<double> generate_walk_positions(double phase);
  std::vector<double> generate_trot_positions(double phase);

  // ── Foot trajectory (swing/stance) ──────────────────────
  double compute_swing_height(double phase, double stride_height) const;
  double compute_stance_position(double phase, double stride_length) const;

  // ── IK helpers ──────────────────────────────────────────
  std::vector<double> leg_ik(
    double x, double y, double z,
    bool is_right_leg) const;

  // ── Callbacks ───────────────────────────────────────────
  void gait_command_callback(const std_msgs::msg::String::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ── State helpers ────────────────────────────────────────
  std::string gait_state_to_string(GaitState state) const;
  void transition_to(GaitState new_state);

  // ── ROS interfaces ───────────────────────────────────────
  rclcpp_lifecycle::LifecyclePublisher<
    trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // ── State ────────────────────────────────────────────────
  GaitState current_state_{GaitState::IDLE};
  double    phase_{0.0};                    // current gait phase [0, 2π]
  double    control_frequency_{50.0};       // Hz
  std::vector<double> current_joint_pos_;   // latest joint states
  bool      stand_published_{false};        // stand trajectory sent (publish-once)

  // ── Parameters ───────────────────────────────────────────
  double hip_length_{0.0838};    // m — GO2 hip offset
  double thigh_length_{0.213};   // m — GO2 thigh link
  double calf_length_{0.213};    // m — GO2 calf link
};

}  // namespace go2_gait_controller

#endif  // GO2_GAIT_CONTROLLER__GAIT_CONTROLLER_NODE_HPP_
