// ============================================================
// go2_gait_controller/gait_controller_node.cpp
// ============================================================
// ROS 2 Lifecycle Node implementation — GO2 Gait State Machine
//
// Author: Yusuf Guenena
// ============================================================

#include "go2_gait_controller/gait_controller_node.hpp"

#include <algorithm>
#include <stdexcept>

namespace go2_gait_controller
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ── Constructor ───────────────────────────────────────────────

GaitControllerNode::GaitControllerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("go2_gait_controller", options)
{
  // Declare parameters with defaults
  this->declare_parameter("control_frequency", 50.0);
  this->declare_parameter("hip_length",        0.0838);
  this->declare_parameter("thigh_length",      0.213);
  this->declare_parameter("calf_length",       0.213);

  RCLCPP_INFO(get_logger(), "GaitControllerNode constructed.");
}

// ── Lifecycle: configure ──────────────────────────────────────

CallbackReturn GaitControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  // Load parameters
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  hip_length_        = this->get_parameter("hip_length").as_double();
  thigh_length_      = this->get_parameter("thigh_length").as_double();
  calf_length_       = this->get_parameter("calf_length").as_double();

  // Initialise joint state vector
  current_joint_pos_.assign(12, 0.0);

  // Create publishers
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_group_effort_controller/joint_trajectory", 10);

  // Create subscribers
  gait_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/go2/gait_command", 10,
    std::bind(&GaitControllerNode::gait_command_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&GaitControllerNode::joint_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Configured. Control frequency: %.1f Hz", control_frequency_);
  return CallbackReturn::SUCCESS;
}

// ── Lifecycle: activate ───────────────────────────────────────

CallbackReturn GaitControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  trajectory_pub_->on_activate();

  // Start control loop timer
  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GaitControllerNode::control_loop, this));

  current_state_ = GaitState::IDLE;
  phase_         = 0.0;

  RCLCPP_INFO(get_logger(), "Active. State: IDLE");
  return CallbackReturn::SUCCESS;
}

// ── Lifecycle: deactivate ─────────────────────────────────────

CallbackReturn GaitControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  control_timer_->cancel();
  trajectory_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

// ── Lifecycle: cleanup ────────────────────────────────────────

CallbackReturn GaitControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  trajectory_pub_.reset();
  gait_cmd_sub_.reset();
  joint_state_sub_.reset();
  control_timer_.reset();
  return CallbackReturn::SUCCESS;
}

// ── Lifecycle: shutdown ───────────────────────────────────────

CallbackReturn GaitControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down.");
  return CallbackReturn::SUCCESS;
}

// ── Control loop ──────────────────────────────────────────────

void GaitControllerNode::control_loop()
{
  trajectory_msgs::msg::JointTrajectory traj;

  switch (current_state_) {
    case GaitState::IDLE:
      // Do nothing — no trajectory sent
      return;

    case GaitState::STAND:
      traj = generate_stand_trajectory();
      break;

    case GaitState::WALK:
      traj = generate_walk_trajectory(phase_);
      phase_ += 2.0 * M_PI * GAIT_PARAMS.at(GaitState::WALK).frequency / control_frequency_;
      if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
      break;

    case GaitState::TROT:
      traj = generate_trot_trajectory(phase_);
      phase_ += 2.0 * M_PI * GAIT_PARAMS.at(GaitState::TROT).frequency / control_frequency_;
      if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
      break;
  }

  trajectory_pub_->publish(traj);
}

// ── Stand trajectory ──────────────────────────────────────────

trajectory_msgs::msg::JointTrajectory
GaitControllerNode::generate_stand_trajectory()
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp    = this->now();
  traj.joint_names     = JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions  = STAND_POSE;
  point.velocities.assign(12, 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);

  traj.points.push_back(point);
  return traj;
}

// ── Walk trajectory ───────────────────────────────────────────
// Walk uses a lateral sequence gait: FL → RR → FR → RL
// Each leg is offset by 90° in phase

trajectory_msgs::msg::JointTrajectory
GaitControllerNode::generate_walk_trajectory(double phase)
{
  const auto & params = GAIT_PARAMS.at(GaitState::WALK);

  // Phase offsets per leg: FR=0, FL=π, RR=π/2, RL=3π/2 (lateral sequence)
  const std::vector<double> offsets = {0.0, M_PI, M_PI_2, 3.0 * M_PI_2};

  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = this->now();
  traj.joint_names  = JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(12);
  point.velocities.assign(12, 0.0);

  for (int leg = 0; leg < 4; ++leg) {
    double leg_phase = std::fmod(phase + offsets[leg], 2.0 * M_PI);
    bool   in_swing  = leg_phase < (2.0 * M_PI * (1.0 - params.stance_ratio));

    double foot_x = compute_stance_position(leg_phase, params.stride_length);
    double foot_z = in_swing
      ? -(params.body_height - compute_swing_height(leg_phase, params.stride_height))
      : -params.body_height;

    bool right = (leg == 0 || leg == 2);  // FR=0, RR=2 are right legs
    auto angles = leg_ik(foot_x, 0.0, foot_z, right);

    int base = leg * 3;
    point.positions[base]     = angles[0];  // hip
    point.positions[base + 1] = angles[1];  // thigh
    point.positions[base + 2] = angles[2];  // calf
  }

  point.time_from_start = rclcpp::Duration::from_seconds(1.0 / control_frequency_);
  traj.points.push_back(point);
  return traj;
}

// ── Trot trajectory ───────────────────────────────────────────
// Trot uses diagonal pairs: (FR + RL) and (FL + RR) alternate

trajectory_msgs::msg::JointTrajectory
GaitControllerNode::generate_trot_trajectory(double phase)
{
  const auto & params = GAIT_PARAMS.at(GaitState::TROT);

  // Diagonal pair offsets: pair 0 (FR+RL) at 0, pair 1 (FL+RR) at π
  // Leg order: FR=0, FL=1, RR=2, RL=3
  const std::vector<double> offsets = {0.0, M_PI, M_PI, 0.0};

  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = this->now();
  traj.joint_names  = JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(12);
  point.velocities.assign(12, 0.0);

  for (int leg = 0; leg < 4; ++leg) {
    double leg_phase = std::fmod(phase + offsets[leg], 2.0 * M_PI);
    bool   in_swing  = leg_phase < (2.0 * M_PI * (1.0 - params.stance_ratio));

    double foot_x = compute_stance_position(leg_phase, params.stride_length);
    double foot_z = in_swing
      ? -(params.body_height - compute_swing_height(leg_phase, params.stride_height))
      : -params.body_height;

    bool right = (leg == 0 || leg == 2);
    auto angles = leg_ik(foot_x, 0.0, foot_z, right);

    int base = leg * 3;
    point.positions[base]     = angles[0];
    point.positions[base + 1] = angles[1];
    point.positions[base + 2] = angles[2];
  }

  point.time_from_start = rclcpp::Duration::from_seconds(1.0 / control_frequency_);
  traj.points.push_back(point);
  return traj;
}

// ── Foot trajectory helpers ───────────────────────────────────

double GaitControllerNode::compute_swing_height(
  double phase, double stride_height) const
{
  // Sinusoidal foot lift during swing phase
  return stride_height * std::sin(phase);
}

double GaitControllerNode::compute_stance_position(
  double phase, double stride_length) const
{
  // Linear foot sweep during stance, reset during swing
  return -stride_length * 0.5 * std::cos(phase);
}

// ── Leg inverse kinematics ────────────────────────────────────
// 3-DOF leg IK for GO2: hip abduction + thigh + calf
// Input:  foot position (x forward, y lateral, z down) in body frame
// Output: [hip_angle, thigh_angle, calf_angle]

std::vector<double> GaitControllerNode::leg_ik(
  double x, double y, double z, bool is_right_leg) const
{
  double hip_sign = is_right_leg ? -1.0 : 1.0;

  // Hip abduction angle
  double hip_angle = hip_sign * std::atan2(y, -z);

  // Project to sagittal plane for thigh + calf IK
  double leg_len_sq = x * x + z * z - hip_length_ * hip_length_;
  double cos_calf = (leg_len_sq - thigh_length_ * thigh_length_ - calf_length_ * calf_length_)
                    / (2.0 * thigh_length_ * calf_length_);
  cos_calf = std::clamp(cos_calf, -1.0, 1.0);

  double calf_angle  = -std::acos(cos_calf);   // negative = knee bent backward
  double thigh_angle = std::atan2(-x, -z)
    - std::atan2(calf_length_ * std::sin(-calf_angle),
                 thigh_length_ + calf_length_ * cos_calf);

  return {hip_angle, thigh_angle, calf_angle};
}

// ── Callbacks ─────────────────────────────────────────────────

void GaitControllerNode::gait_command_callback(
  const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & cmd = msg->data;

  if      (cmd == "idle")  transition_to(GaitState::IDLE);
  else if (cmd == "stand") transition_to(GaitState::STAND);
  else if (cmd == "walk")  transition_to(GaitState::WALK);
  else if (cmd == "trot")  transition_to(GaitState::TROT);
  else {
    RCLCPP_WARN(get_logger(), "Unknown gait command: '%s'", cmd.c_str());
  }
}

void GaitControllerNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store latest joint positions for feedback
  if (msg->position.size() >= 12) {
    current_joint_pos_ = std::vector<double>(
      msg->position.begin(), msg->position.begin() + 12);
  }
}

// ── State helpers ─────────────────────────────────────────────

void GaitControllerNode::transition_to(GaitState new_state)
{
  if (new_state == current_state_) return;

  RCLCPP_INFO(get_logger(),
    "Gait transition: %s → %s",
    gait_state_to_string(current_state_).c_str(),
    gait_state_to_string(new_state).c_str());

  // Reset phase on any transition
  phase_         = 0.0;
  current_state_ = new_state;
}

std::string GaitControllerNode::gait_state_to_string(GaitState state) const
{
  switch (state) {
    case GaitState::IDLE:  return "IDLE";
    case GaitState::STAND: return "STAND";
    case GaitState::WALK:  return "WALK";
    case GaitState::TROT:  return "TROT";
    default:               return "UNKNOWN";
  }
}

}  // namespace go2_gait_controller
