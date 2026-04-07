// ============================================================
// go2_gait_controller/gait_controller_node.cpp
// ============================================================
// ROS 2 Lifecycle Node implementation — GO2 Gait State Machine
// Uses ForwardCommandController (position interface) for
// jitter-free Gazebo simulation.
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
  this->declare_parameter("control_frequency", 50.0);
  this->declare_parameter("hip_length",        0.0838);
  this->declare_parameter("thigh_length",      0.213);
  this->declare_parameter("calf_length",       0.213);
  this->declare_parameter("auto_activate",     true);

  RCLCPP_INFO(get_logger(), "GaitControllerNode constructed.");

  bool auto_activate = this->get_parameter("auto_activate").as_bool();
  if (auto_activate) {
    RCLCPP_INFO(get_logger(), "Auto-activating (auto_activate=true)...");
    auto cfg_ret = on_configure(get_current_state());
    if (cfg_ret == CallbackReturn::SUCCESS) {
      auto act_ret = on_activate(get_current_state());
      if (act_ret == CallbackReturn::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Auto-activation complete.");
      }
    }
  }
}

// ── Lifecycle: configure ──────────────────────────────────────

CallbackReturn GaitControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  control_frequency_ = this->get_parameter("control_frequency").as_double();
  hip_length_        = this->get_parameter("hip_length").as_double();
  thigh_length_      = this->get_parameter("thigh_length").as_double();
  calf_length_       = this->get_parameter("calf_length").as_double();

  current_joint_pos_.assign(12, 0.0);

  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_group_effort_controller/joint_trajectory", 10);

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

  auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GaitControllerNode::control_loop, this));

  current_state_   = GaitState::STAND;
  phase_           = 0.0;
  stand_published_ = false;

  RCLCPP_INFO(get_logger(), "Active. State: STAND (auto-hold)");
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
  std::vector<double> positions;

  switch (current_state_) {
    case GaitState::IDLE:
      return;

    case GaitState::STAND:
      positions = generate_stand_positions();
      break;

    case GaitState::WALK:
      positions = generate_walk_positions(phase_);
      phase_ += 2.0 * M_PI * GAIT_PARAMS.at(GaitState::WALK).frequency / control_frequency_;
      if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
      break;

    case GaitState::TROT:
      positions = generate_trot_positions(phase_);
      phase_ += 2.0 * M_PI * GAIT_PARAMS.at(GaitState::TROT).frequency / control_frequency_;
      if (phase_ > 2.0 * M_PI) phase_ -= 2.0 * M_PI;
      break;
  }

  // Wrap in JointTrajectory for trajectory controller
  trajectory_msgs::msg::JointTrajectory traj;
  // stamp=0 + time_from_start=0 = execute at current time
  traj.joint_names     = JOINT_NAMES;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions      = positions;
  point.velocities.assign(12, 0.0);

  traj.points.push_back(point);
  trajectory_pub_->publish(traj);
}

// ── Stand positions ──────────────────────────────────────────

std::vector<double>
GaitControllerNode::generate_stand_positions()
{
  return STAND_POSE;
}

// ── Walk positions ───────────────────────────────────────────

std::vector<double>
GaitControllerNode::generate_walk_positions(double phase)
{
  const auto & params = GAIT_PARAMS.at(GaitState::WALK);
  const std::vector<double> offsets = {0.0, M_PI, M_PI_2, 3.0 * M_PI_2};

  std::vector<double> positions(12);

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
    positions[base]     = angles[0];
    positions[base + 1] = angles[1];
    positions[base + 2] = angles[2];
  }

  return positions;
}

// ── Trot positions ───────────────────────────────────────────

std::vector<double>
GaitControllerNode::generate_trot_positions(double phase)
{
  const auto & params = GAIT_PARAMS.at(GaitState::TROT);
  const std::vector<double> offsets = {0.0, M_PI, M_PI, 0.0};

  std::vector<double> positions(12);

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
    positions[base]     = angles[0];
    positions[base + 1] = angles[1];
    positions[base + 2] = angles[2];
  }

  return positions;
}

// ── Foot trajectory helpers ───────────────────────────────────

double GaitControllerNode::compute_swing_height(
  double phase, double stride_height) const
{
  return stride_height * std::sin(phase);
}

double GaitControllerNode::compute_stance_position(
  double phase, double stride_length) const
{
  return -stride_length * 0.5 * std::cos(phase);
}

// ── Leg inverse kinematics ────────────────────────────────────

std::vector<double> GaitControllerNode::leg_ik(
  double x, double y, double z, bool is_right_leg) const
{
  double hip_sign = is_right_leg ? -1.0 : 1.0;

  double hip_angle = hip_sign * std::atan2(y, -z);

  double leg_len_sq = x * x + z * z - hip_length_ * hip_length_;
  double cos_calf = (leg_len_sq - thigh_length_ * thigh_length_ - calf_length_ * calf_length_)
                    / (2.0 * thigh_length_ * calf_length_);
  cos_calf = std::clamp(cos_calf, -1.0, 1.0);

  double calf_angle  = -std::acos(cos_calf);
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
    "Gait transition: %s -> %s",
    gait_state_to_string(current_state_).c_str(),
    gait_state_to_string(new_state).c_str());

  phase_           = 0.0;
  stand_published_ = false;
  current_state_   = new_state;
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
