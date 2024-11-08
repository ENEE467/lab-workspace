#include <filesystem>

#include <matplot/matplot.h>

#include "lab8/eef_pose_tracker.hpp"

EEFPoseTracker::EEFPoseTracker()
: Node ("eef_pose_tracker")
{
  using namespace std::chrono_literals;

  track_service_ = this->create_service<lab8::srv::TrackRequest>(
    "track_eef",
    std::bind(&EEFPoseTracker::requestCallback, this, std::placeholders::_1, std::placeholders::_2));

  timer_ = this->create_wall_timer(500ms, std::bind(&EEFPoseTracker::timerCallback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto param_description {rcl_interfaces::msg::ParameterDescriptor()};
  param_description.description = "Sets the workspace directory for saving the plots and CSV files";
  this->declare_parameter("workspace_dir", "", param_description);
}

void EEFPoseTracker::requestCallback(
  const std::shared_ptr<lab8::srv::TrackRequest::Request> request,
  std::shared_ptr<lab8::srv::TrackRequest::Response> response)
{
  if (should_track_ == request->status) {
    RCLCPP_WARN(
      this->get_logger(),
      "\n\nEnd-effector tracking is currently in the same state as requested\n\n");

    response->set__success(false);

    return;
  }

  should_track_ = request->status;

  if (!should_track_) {
    RCLCPP_INFO(this->get_logger(), "\n\nEnd-effector tracking stops now...\n\n");

    plotData();
    if (poses_csv_.is_open()) {poses_csv_.close();}
    clearData();
    response->set__success(true);

    return;
  }

  if (request->tf_root_frame_name == request->tf_tip_frame_name) {
    RCLCPP_WARN(this->get_logger(), "\n\nRoot and tip frames of the arm cannot be the same\n\n");

    should_track_ = false;
    response->set__success(false);

    return;
  }

  if (request->plot_axis_x == request->plot_axis_y) {
    RCLCPP_WARN(this->get_logger(), "\n\nData for X and Y axes in the plot cannot be the same\n\n");

    should_track_ = false;
    response->set__success(false);

    return;
  }

  workspace_dir_ = this->get_parameter("workspace_dir").as_string();

  if (!std::filesystem::is_directory(workspace_dir_)) {
    RCLCPP_WARN(this->get_logger(), "\n\nA correct workspace directory isn't set\n\n");

    should_track_ = false;
    response->set__success(false);

    return;
  }

  if (!std::filesystem::is_directory(workspace_dir_ + "/output/lab8"))
    std::filesystem::create_directories(workspace_dir_ + "/output/lab8");

  output_timestamp_ = createTimeStamp();
  poses_csv_.open(workspace_dir_ + "/output/lab8/poses-" + output_timestamp_ + ".csv");

  from_frame_ = request->tf_root_frame_name;
  to_frame_ = request->tf_tip_frame_name;
  plot_title_ = request->plot_title;
  plot_axis_x_ = request->plot_axis_x;
  plot_axis_y_ = request->plot_axis_y;

  RCLCPP_INFO(this->get_logger(), "\n\nEnd-effector tracking starts now...\n\n");
  response->set__success(true);
}

void EEFPoseTracker::timerCallback()
{
  try {
    transform_ = tf_buffer_->lookupTransform(from_frame_, to_frame_, tf2::TimePointZero).transform;
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Tried to transform " << from_frame_ << " to " << to_frame_  << " : " << ex.what());

    return;
  }

  if (!should_track_)
    return;

  x_points_.push_back(transform_.translation.x);
  y_points_.push_back(transform_.translation.y);
  z_points_.push_back(transform_.translation.z);

  if (!poses_csv_.is_open())
    return;

  poses_csv_
    << transform_.translation.x << ", "
    << transform_.translation.y << ", "
    << transform_.translation.z << ", "
    << transform_.rotation.x << ", "
    << transform_.rotation.y << ", "
    << transform_.rotation.z << ", "
    << transform_.rotation.w << ", "
    << '\n';
}

void EEFPoseTracker::plotData()
{
  std::vector<double> axis_x_points {};
  std::vector<double> axis_y_points {};

  std::string axis_x_label {};
  std::string axis_y_label {};

  switch (plot_axis_x_) {

  case lab8::srv::TrackRequest::Request::X_AXIS:
    axis_x_points = x_points_;
    axis_x_label = "X Axis";
    break;

  case lab8::srv::TrackRequest::Request::Y_AXIS:
    axis_x_points = y_points_;
    axis_x_label = "Y Axis";
    break;

  case lab8::srv::TrackRequest::Request::Z_AXIS:
    axis_x_points = z_points_;
    axis_x_label = "Z Axis";
    break;

  default:
    break;

  }

  switch (plot_axis_y_) {

  case lab8::srv::TrackRequest::Request::X_AXIS:
    axis_y_points = x_points_;
    axis_y_label = "X Axis";
    break;

  case lab8::srv::TrackRequest::Request::Y_AXIS:
    axis_y_points = y_points_;
    axis_y_label = "Y Axis";
    break;

  case lab8::srv::TrackRequest::Request::Z_AXIS:
    axis_y_points = z_points_;
    axis_y_label = "Z Axis";
    break;

  default:
    break;

  }

  matplot::title(plot_title_);
  matplot::xlabel(axis_x_label);
  matplot::ylabel(axis_y_label);
  matplot::plot(axis_x_points, axis_y_points);

  matplot::save(workspace_dir_ + "/output/lab8/plot-" + output_timestamp_ + ".png");
}

std::string EEFPoseTracker::createTimeStamp()
{
  std::stringstream timeStamp;

  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  timeStamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

  return timeStamp.str();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto tracker_node {std::make_shared<EEFPoseTracker>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
