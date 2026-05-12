/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <industrial_robot_client/joint_trajectory_action.h>
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <rclcpp/rclcpp.hpp>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{

const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction::JointTrajectoryAction() :
    has_active_goal_(false), controller_alive_(false), has_moved_once_(false),
    name_("joint_trajectory_action")
{
  node_ = rclcpp::Node::make_shared("joint_trajectory_action");

  node_->declare_parameter("constraints/goal_threshold", DEFAULT_GOAL_THRESHOLD_);
  goal_threshold_ = node_->get_parameter("constraints/goal_threshold").as_double();

  if (!industrial_utils::param::getJointNames(node_, "controller_joint_names", "robot_description", joint_names_))
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize joint_names.");

  // The controller joint names parameter includes empty joint names for those joints not supported
  // by the controller.  These are removed since the trajectory action should ignore these.
  std::remove(joint_names_.begin(), joint_names_.end(), std::string());
  RCLCPP_INFO(node_->get_logger(), "Filtered joint names to %zu joints", joint_names_.size());

  pub_trajectory_command_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_path_command", 1);
  sub_trajectory_state_ = node_->create_subscription<control_msgs::action::FollowJointTrajectory::Feedback>(
    "feedback_states", 1,
    std::bind(&JointTrajectoryAction::controllerStateCB, this, std::placeholders::_1));
  sub_robot_status_ = node_->create_subscription<industrial_msgs::msg::RobotStatus>(
    "robot_status", 1,
    std::bind(&JointTrajectoryAction::robotStatusCB, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
    node_,
    "joint_trajectory_action",
    std::bind(&JointTrajectoryAction::goalCB, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&JointTrajectoryAction::cancelCB, this, std::placeholders::_1),
    std::bind(&JointTrajectoryAction::acceptedCB, this, std::placeholders::_1));

  watchdog_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(WATCHDOG_PERIOD_),
    std::bind(&JointTrajectoryAction::watchdog, this));
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

void JointTrajectoryAction::robotStatusCB(const industrial_msgs::msg::RobotStatus::SharedPtr msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
  has_moved_once_ = has_moved_once_ ? true : (last_robot_status_->in_motion.val == industrial_msgs::msg::TriState::TRUE);
}

void JointTrajectoryAction::watchdog()
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Waiting for subscription to joint trajectory state");
  }

  RCLCPP_WARN(node_->get_logger(), "Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
  controller_alive_ = false;

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    // last_trajectory_state_ is null if the subscriber never makes a connection
    if (!last_trajectory_state_)
    {
      RCLCPP_WARN(node_->get_logger(), "Aborting goal because we have never heard a controller state message.");
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(),
          "Aborting goal because we haven't heard from the controller in %f seconds", WATCHDOG_PERIOD_);
    }

    abortGoal();
  }
}

rclcpp_action::GoalResponse JointTrajectoryAction::goalCB(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(node_->get_logger(), "Received new goal");

  // reject all goals as long as we haven't heard from the remote controller
  if (!controller_alive_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint trajectory action rejected: waiting for (initial) feedback from controller");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->trajectory.points.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint trajectory action failed on empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!industrial_utils::isSimilar(joint_names_, goal->trajectory.joint_names))
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint trajectory action failing on invalid joints");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (rclcpp::Duration(goal->goal_time_tolerance).seconds() > 0.0)
  {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!goal->goal_tolerance.empty())
  {
    RCLCPP_WARN(node_->get_logger(),
        "Ignoring goal tolerance in action, using paramater tolerance of %f instead", goal_threshold_);
  }
  if (!goal->path_tolerance.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryAction::cancelCB(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> gh)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    trajectory_msgs::msg::JointTrajectory empty;
    empty.joint_names = joint_names_;
    pub_trajectory_command_->publish(empty);

    // Marks the current goal as canceled.
    active_goal_->canceled(std::make_shared<control_msgs::action::FollowJointTrajectory::Result>());
    has_active_goal_ = false;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Active goal and goal cancel do not match, ignoring cancel request");
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryAction::acceptedCB(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> gh)
{
  // Cancels the currently active goal.
  if (has_active_goal_)
  {
    RCLCPP_WARN(node_->get_logger(), "Received new goal, canceling current goal");
    abortGoal();
  }

  active_goal_ = gh;
  has_active_goal_ = true;
  time_to_check_ = node_->now() +
      rclcpp::Duration::from_seconds(
        active_goal_->get_goal()->trajectory.points.back().time_from_start.sec / 2.0);
  has_moved_once_ = false;

  RCLCPP_INFO(node_->get_logger(), "Publishing trajectory");

  current_traj_ = active_goal_->get_goal()->trajectory;
  pub_trajectory_command_->publish(current_traj_);
}

void JointTrajectoryAction::controllerStateCB(const control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "Checking controller state feedback");

  last_trajectory_state_ = msg;
  controller_alive_ = true;

  watchdog_timer_->reset();

  if (!has_active_goal_)
  {
    //RCLCPP_DEBUG(node_->get_logger(), "No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint names from the controller don't match our joint names.");
    return;
  }

  if (!has_moved_once_ && (node_->now() < time_to_check_))
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting to check for goal completion until halfway through trajectory");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  RCLCPP_DEBUG(node_->get_logger(), "Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::msg::TriState::FALSE)
      {
        RCLCPP_INFO(node_->get_logger(), "Inside goal constraints - stopped moving-  return success for action");
        active_goal_->succeed(std::make_shared<control_msgs::action::FollowJointTrajectory::Result>());
        has_active_goal_ = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::msg::TriState::UNKNOWN)
      {
        RCLCPP_INFO(node_->get_logger(), "Inside goal constraints, return success for action");
        RCLCPP_WARN(node_->get_logger(), "Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_->succeed(std::make_shared<control_msgs::action::FollowJointTrajectory::Result>());
        has_active_goal_ = false;
      }
      else
      {
        RCLCPP_DEBUG(node_->get_logger(), "Within goal constraints but robot is still moving");
      }
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Inside goal constraints, return success for action");
      RCLCPP_WARN(node_->get_logger(), "Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_->succeed(std::make_shared<control_msgs::action::FollowJointTrajectory::Result>());
      has_active_goal_ = false;
    }
  }
}

void JointTrajectoryAction::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::msg::JointTrajectory empty;
  pub_trajectory_command_->publish(empty);

  // Marks the current goal as aborted.
  active_goal_->abort(std::make_shared<control_msgs::action::FollowJointTrajectory::Result>());
  has_active_goal_ = false;
}

bool JointTrajectoryAction::withinGoalConstraints(const control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr &msg,
                                                  const trajectory_msgs::msg::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    if (industrial_robot_client::utils::isWithinRange(last_trajectory_state_->joint_names,
                                                      last_trajectory_state_->actual.positions, traj.joint_names,
                                                      traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

} //joint_trajectory_action
} //industrial_robot_client
