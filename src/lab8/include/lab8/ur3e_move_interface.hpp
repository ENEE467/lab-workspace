#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lab8/srv/track_request.hpp"

class UR3eMoveInterface : public rclcpp::Node {

public:
  /**
   * @brief Constructs a new `ur3e_move_interface` node
   *
   * @param node_options
   */
  UR3eMoveInterface(const rclcpp::NodeOptions& node_options);

  /**
   * @brief Demonstrations for moving the UR3e arm using the MoveGroup Interface.
   */
  void examplesMoveIt();

  /**
   * @brief Moves the UR3e arm to draw a circle in the XY plane (Horizontal)
   *
   * @param radius_meters Radius in meters
   */
  void drawCircleXY(double radius_meters = 0);

  /**
   * @brief Moves the UR3e arm to draw a circle in the YZ plane (Vertical)
   *
   * @param radius_meters Radius in meters
   */
  void drawCircleYZ(double radius_meters = 0);

  /**
   * @brief Moves the UR3e arm to draw a square in the XY plane (Horizontal)
   *
   * @param side_meters Side length in meters
   */
  void drawSquareXY(double side_meters = 0);

  /**
   * @brief Moves the UR3e arm to draw a square in the YZ plane (Vertical)
   *
   * @param side_meters Side length in meters
   */
  void drawSquareYZ(double side_meters = 0);

private:
  /**
   * @brief Gets the robot base pose relative to world frame
   *
   * @return geometry_msgs::msg::Pose
   */
  geometry_msgs::msg::Pose getRobotBasePose();

  /**
   * @brief Plans and generates a trajectory to move to a joint-space target.
   *
   * @param target_joint_positions Vector of joint positions in radians
   * @param motion_plan Object to which the generated plan will be written to.
   * @return true
   * @return false
   */
  bool planToJointSpaceGoal(
    const std::vector<double>& target_joint_positions,
    moveit::planning_interface::MoveGroupInterface::Plan& motion_plan);

  /**
   * @brief Plans and generates a trajectory to move through a bunch of waypoints given in the
   *        Cartesian space.
   *
   * @param waypoints
   * @param trajectory
   * @return true
   * @return false
   */
  bool planCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory);

  /**
   * @brief Plans and generates a trajectory to move to an end-effector pose given in Cartesian
   *        space.
   *
   * @param pose_goal
   * @param motion_plan
   * @return true
   * @return false
   */
  bool planToPoseGoal(
    const geometry_msgs::msg::Pose& pose_goal,
    moveit::planning_interface::MoveGroupInterface::Plan& motion_plan);

  /**
   * @brief Plans and generates a trajectory to move the arm to a preset, named position.
   *
   * @param name
   * @param motion_plan
   * @return true
   * @return false
   */
  bool planToNamedTarget(
    const std::string& name,
    moveit::planning_interface::MoveGroupInterface::Plan& motion_plan);

  /**
   * @brief Sends a request to start tracking the end-effector poses for plotting and saving.
   *
   * @param request
   * @return true
   * @return false
   */
  bool sendEEFTrackRequest(const lab8::srv::TrackRequest::Request::SharedPtr& request);

  bool tracking_service_available_ {false};

  double velocity_scaling_factor_ {0.1};
  double acceleration_scaling_factor_ {0.1};

  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ {nullptr};

  rclcpp::Client<lab8::srv::TrackRequest>::SharedPtr track_eef_client_ {nullptr};

  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_ {nullptr};
  trajectory_processing::TimeOptimalTrajectoryGeneration time_optimal_trajectory_generation_;

};
