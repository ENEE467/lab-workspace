#include <tf2/LinearMath/Quaternion.h>

#include "lab8/ur3e_move_interface.hpp"

UR3eMoveInterface::UR3eMoveInterface(const rclcpp::NodeOptions& node_options)
: Node {"ur3e_move_interface", node_options},
  move_group_interface_ {std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    rclcpp::Node::SharedPtr(std::move(this)), "ur_manipulator")}
{
  RCLCPP_INFO(this->get_logger(), "Initializing move group node...");
  move_group_interface_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
  move_group_interface_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);

  move_group_interface_->startStateMonitor(3);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Waiting for end-effector tracking service");
  track_eef_client_ = this->create_client<lab8::srv::TrackRequest>("track_eef");
  tracking_service_available_ = track_eef_client_->wait_for_service(std::chrono::seconds(3));

  if (!tracking_service_available_)
    RCLCPP_WARN(this->get_logger(), "Tracking service is unavailable, poses cannot be saved/plotted.");
}

geometry_msgs::msg::Pose UR3eMoveInterface::getRobotBasePose()
{
  geometry_msgs::msg::Pose pose_out;

  geometry_msgs::msg::Transform transform_robot_base_world {};
  std::string world_frame_name {move_group_interface_->getPlanningFrame()};
  std::string robot_base_frame_name {move_group_interface_->getLinkNames().front()};

  try {
    transform_robot_base_world =
      tf_buffer_->lookupTransform(
        world_frame_name, robot_base_frame_name, tf2::TimePointZero).transform;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Tried to transform " << world_frame_name << " to " << robot_base_frame_name  << " : "
                            << ex.what());

    return pose_out;
  }

  pose_out.position.x = transform_robot_base_world.translation.x;
  pose_out.position.y = transform_robot_base_world.translation.y;
  pose_out.position.z = transform_robot_base_world.translation.z;
  pose_out.orientation = transform_robot_base_world.rotation;

  return pose_out;
}

bool UR3eMoveInterface::planToJointSpaceGoal(
  const std::vector<double>& target_joint_positions,
  moveit::planning_interface::MoveGroupInterface::Plan& motion_plan)
{
  bool within_bounds {move_group_interface_->setJointValueTarget(target_joint_positions)};

  if (!within_bounds) {
    RCLCPP_WARN_STREAM(this->get_logger(),
    "Some target joint positions were outside the limits. " <<
    "Generated motion plan will be clamped to the joint limits.");
  }

  bool plan_success {
    move_group_interface_->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS};

  if (plan_success)
    RCLCPP_INFO(this->get_logger(), "Plan to target joint positions succeeded");
  else
    RCLCPP_ERROR(this->get_logger(), "Plan to target joint positions failed");

  return plan_success;
}

bool UR3eMoveInterface::planToPoseGoal(
  const geometry_msgs::msg::Pose& pose_goal,
  moveit::planning_interface::MoveGroupInterface::Plan& motion_plan)
{
  move_group_interface_->setPoseTarget(pose_goal);

  bool plan_success {
    move_group_interface_->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS};

  if (plan_success)
    RCLCPP_INFO(this->get_logger(), "Plan to target end-effector pose succeeded");
  else
    RCLCPP_ERROR(this->get_logger(), "Plan to target end-effector pose failed");

  return plan_success;
}

bool UR3eMoveInterface::planToNamedTarget(
  const std::string& name,
  moveit::planning_interface::MoveGroupInterface::Plan& motion_plan) {

  move_group_interface_->setNamedTarget(name);

  bool plan_success {
    move_group_interface_->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS};

  if (plan_success)
    RCLCPP_INFO(this->get_logger(), "Plan to target end-effector pose succeeded");
  else
    RCLCPP_ERROR(this->get_logger(), "Plan to target end-effector pose failed");

  return plan_success;
}

bool UR3eMoveInterface::planCartesianPath(
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  moveit_msgs::msg::RobotTrajectory& trajectory)
{
  const double jump_threshold {0.0};
  const double eef_step {0.01};
  double fraction {
    move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory)};

  RCLCPP_INFO_STREAM(this->get_logger(), "Planned cartesian path: " << fraction*100 << "%");

  bool plan_success {fraction*100 > 95.0};

  if (plan_success)
    RCLCPP_INFO(this->get_logger(), "Plan to follow a cartesian path succeeded");
  else
    RCLCPP_WARN_STREAM(this->get_logger(), "Couldn't fully achieve planned cartesian path");

  if (!plan_success)
    return plan_success;

  robot_trajectory::RobotTrajectory robot_trajectory {
    move_group_interface_->getCurrentState()->getRobotModel(), move_group_interface_->getName()};

  robot_trajectory.setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), trajectory);
  time_optimal_trajectory_generation_.computeTimeStamps(
    robot_trajectory,
    velocity_scaling_factor_, acceleration_scaling_factor_);
  robot_trajectory.getRobotTrajectoryMsg(trajectory);

  return plan_success;
}

bool UR3eMoveInterface::sendEEFTrackRequest(const lab8::srv::TrackRequest::Request::SharedPtr& request)
{
  if (!tracking_service_available_) {
    RCLCPP_WARN(
      this->get_logger(), "EEF tracking request send failed because this service is unavailable.");

    return false;
  }

  auto result_future {track_eef_client_->async_send_request(request).share()};

  RCLCPP_INFO(this->get_logger(), "EEF tracking request sent, waiting for result...");
  auto result_status {result_future.wait_for(std::chrono::seconds(1))};

  if (result_future.get()->success && result_status == std::future_status::ready)
    RCLCPP_INFO(this->get_logger(), "EEF tracking request success.");
  else
    RCLCPP_WARN(this->get_logger(), "EEF tracking request failed.");

  return result_future.get()->success;
}

int main(int argc, char** argv)
{
  std::string shape_argument {};
  std::string orientation_argument {};
  std::string size_argument {};

  if (argv[1])
    shape_argument = argv[1];

  if (argv[2])
    orientation_argument = argv[2];

  if (argv[3])
    size_argument = argv[3];

  double size {};
  try {
    size = std::stod(size_argument);
  }
  catch (std::exception& exception) {
    std::cout << '\n' << "A valid size value isn't given, default will be used instead." << '\n'
              << '\n';
    size = 0;
  }

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto mover_node {std::make_shared<UR3eMoveInterface>(node_options)};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mover_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  if (shape_argument == "circle" && orientation_argument == "horizontal")
    mover_node->drawCircleXY(size);

  else if (shape_argument == "circle" && orientation_argument == "vertical")
    mover_node->drawCircleYZ(size);

  else if (shape_argument == "square" && orientation_argument == "horizontal")
    mover_node->drawSquareXY(size);

  else if (shape_argument == "square" && orientation_argument == "vertical")
    mover_node->drawSquareYZ(size);

  else
    mover_node->examplesMoveIt();

  rclcpp::shutdown();

  return 0;
}
