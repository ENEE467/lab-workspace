#include "lab8/ur3e_move_interface.hpp"

void UR3eMoveInterface::drawCircleXY(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45;

  double angle_offset {-M_PI_2};

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  {
    std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, 0};
    move_group_interface_->setJointValueTarget(target_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan)};

    if (plan_success)
      move_group_interface_->execute(motion_plan);
  }

  {
    auto current_pose {move_group_interface_->getCurrentPose().pose};

    auto target_offset_adjust_pose {current_pose};
    target_offset_adjust_pose.position.x = radius_meters * std::cos(angle_offset);
    target_offset_adjust_pose.position.y = radius_meters * std::sin(angle_offset);

    auto target_z_adjust_pose {target_offset_adjust_pose};
    target_z_adjust_pose.position.z = getRobotBasePose().position.z + 0.2;

    std::vector<geometry_msgs::msg::Pose> waypoints {
      current_pose, target_offset_adjust_pose, target_z_adjust_pose};

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  {
    auto initial_pose {move_group_interface_->getCurrentPose().pose};

    std::vector<geometry_msgs::msg::Pose> waypoints {};
    waypoints.push_back(initial_pose);

    for (double theta {0 + angle_offset}; theta < 2 * M_PI + angle_offset; theta += M_PI / 180) {
      geometry_msgs::msg::Pose point_on_shape {initial_pose};

      point_on_shape.position.x = radius_meters * std::cos(theta);
      point_on_shape.position.y = radius_meters * std::sin(theta);

      waypoints.push_back(point_on_shape);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Move the robot back to its home position by setting a named target
  {
    move_group_interface_->setNamedTarget("up");
    move_group_interface_->move();
  }
}

void UR3eMoveInterface::drawCircleYZ(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.45 * 0.5;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  {
    std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};
    move_group_interface_->setJointValueTarget(target_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan)};

    if (plan_success)
      move_group_interface_->execute(motion_plan);
  }

  {
    auto current_pose {move_group_interface_->getCurrentPose().pose};
    auto robot_base_pose {getRobotBasePose()};

    auto target_adjust_pose {current_pose};
    target_adjust_pose.position.y = robot_base_pose.position.y;

    double lower_z_bound {getRobotBasePose().position.z + 0.15};
    double upper_z_bound {lower_z_bound + 2 * radius_meters};
    target_adjust_pose.position.z = (lower_z_bound + upper_z_bound) / 2;

    std::vector<geometry_msgs::msg::Pose> waypoints {current_pose, target_adjust_pose};

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto initial_pose {move_group_interface_->getCurrentPose().pose};
    waypoints.push_back(initial_pose);

    for (double theta {0}; theta < 2 * M_PI; theta += M_PI / 180) {
      geometry_msgs::msg::Pose point_on_shape {initial_pose};

      point_on_shape.position.y += radius_meters * std::cos(theta);
      point_on_shape.position.z += radius_meters * std::sin(theta);

      waypoints.push_back(point_on_shape);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Move the robot back to its home position by setting a named target
  {
    move_group_interface_->setNamedTarget("up");
    move_group_interface_->move();
  }
}

void UR3eMoveInterface::drawSquareXY(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) * M_SQRT1_2;

  double angle_offset {-M_PI_2};

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  {
    std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, 0};
    move_group_interface_->setJointValueTarget(target_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan)};

    if (plan_success)
      move_group_interface_->execute(motion_plan);
  }

  {
    auto current_pose {move_group_interface_->getCurrentPose().pose};

    auto target_offset_adjust_pose {current_pose};
    target_offset_adjust_pose.position.x = 0.5 * M_SQRT2 * side_meters * std::cos(angle_offset);
    target_offset_adjust_pose.position.y = 0.5 * M_SQRT2 * side_meters * std::sin(angle_offset);

    auto target_z_adjust_pose {target_offset_adjust_pose};
    target_z_adjust_pose.position.z = getRobotBasePose().position.z + 0.2;

    std::vector<geometry_msgs::msg::Pose> waypoints {
      current_pose, target_offset_adjust_pose, target_z_adjust_pose};

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan
  {
    auto initial_pose {move_group_interface_->getCurrentPose().pose};

    std::vector<geometry_msgs::msg::Pose> waypoints {};
    waypoints.push_back(initial_pose);

    for (double theta {0 + angle_offset}; theta < 2 * M_PI + angle_offset; theta += M_PI / 180) {
      geometry_msgs::msg::Pose point_on_shape {initial_pose};

      double x_point {0.5 * M_SQRT2 * side_meters * std::cos(theta)};
      double y_point {0.5 * M_SQRT2 * side_meters * std::sin(theta)};

      point_on_shape.position.x = std::clamp(x_point, -(0.5 * side_meters), 0.5 * side_meters);
      point_on_shape.position.y = std::clamp(y_point, -(0.5 * side_meters), 0.5 * side_meters);

      waypoints.push_back(point_on_shape);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Move the robot back to its home position by setting a named target
  {
    move_group_interface_->setNamedTarget("up");
    move_group_interface_->move();
  }
}

void UR3eMoveInterface::drawSquareYZ(double side_meters)
{
  if (side_meters <= 0)
    side_meters = 0.425;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  {
    std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};
    move_group_interface_->setJointValueTarget(target_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan)};

    if (plan_success)
      move_group_interface_->execute(motion_plan);
  }

  {
    auto current_pose {move_group_interface_->getCurrentPose().pose};
    auto robot_base_pose {getRobotBasePose()};

    auto target_adjust_pose {current_pose};
    target_adjust_pose.position.y = robot_base_pose.position.y;

    double lower_z_bound {getRobotBasePose().position.z + 0.15};
    double upper_z_bound {lower_z_bound + side_meters};
    target_adjust_pose.position.z = (lower_z_bound + upper_z_bound) / 2;

    std::vector<geometry_msgs::msg::Pose> waypoints {current_pose, target_adjust_pose};

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto initial_pose {move_group_interface_->getCurrentPose().pose};
    waypoints.push_back(initial_pose);

    for (double theta {0}; theta < 2 * M_PI; theta += M_PI / 180) {
      geometry_msgs::msg::Pose point_on_shape {initial_pose};

      double y_displacement {0.5 * M_SQRT2 * side_meters * std::cos(theta)};
      double z_displacement {0.5 * M_SQRT2 * side_meters * std::sin(theta)};

      point_on_shape.position.y += std::clamp(y_displacement, -(0.5 * side_meters), 0.5 * side_meters);
      point_on_shape.position.z += std::clamp(z_displacement, -(0.5 * side_meters), 0.5 * side_meters);

      waypoints.push_back(point_on_shape);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Move the robot back to its home position by setting a named target
  {
    move_group_interface_->setNamedTarget("up");
    move_group_interface_->move();
  }
}
