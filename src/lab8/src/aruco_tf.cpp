#include "../include/aruco_tf.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <aruco_opencv_msgs/msg/detail/aruco_detection__struct.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
// Test Node
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      this->declare_parameter("mine", "hello");

      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));

      subscriber_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>("aruco_detections", 10, std::bind(&MinimalPublisher::callback, this, std::placeholders::_1));
    }

  private:
    void callback(const aruco_opencv_msgs::msg::ArucoDetection & msg) const {
      aruco_opencv_msgs::msg::ArucoDetection detect = msg;
      RCLCPP_INFO(this->get_logger(), "Hmmm");
    }
    void timer_callback()
    {
      std::string param = this->get_parameter("mine").as_string();

      auto message = std_msgs::msg::String();
      message.data = ", world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s%s'", param.c_str(), message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr subscriber_;
    aruco_opencv_msgs::msg::ArucoDetection aruco_detection;
    };



// ArucoTF Functions
/**
 * @brief Function to save calibration data to file
 * 
 * @param save_rot Rotation extrinsic
 * @param save_trans Translation extrinsic
 */
void ArucoTF::saveCalibToFile(const Eigen::Quaternionf &save_rot, const Eigen::Vector3f &save_trans){
  if (!ArucoTF::calib){
    RCLCPP_INFO(this->get_logger(), "Saving calibration to file");

    std::string calib_path = ament_index_cpp::get_package_share_directory("lab8");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    auto index = calib_path.find("install");
    auto actual_path = calib_path.substr(0, index);
    actual_path += "src/lab8/calibration/camera/logitech_extrinsics.json";

    std::vector<float> rot, trans;

    // Convert quaternion (w,x,y,z) from Eigen to Vector
    rot.push_back(save_rot.w());
    rot.push_back(save_rot.x());
    rot.push_back(save_rot.y());
    rot.push_back(save_rot.z());

    // Convert translation from Eigen to Vector
    trans.push_back(save_trans(0));
    trans.push_back(save_trans(1));
    trans.push_back(save_trans(2));

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json object
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      RCLCPP_WARN(this->get_logger(), "JSON Parse failed: %s", e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section = "logitech_webcam";
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
    }

    // Add rotation and translation to json object
    calib_data[calib_section]["rot"] = rot;
    calib_data[calib_section]["trans"] = trans;

    std::ofstream calib_file_out(calib_path,
                                 std::fstream::out | std::ofstream::trunc);
    calib_file_out << std::setprecision(16) << calib_data;
    calib_file_out.close();

    std::ofstream actual_file_out(actual_path,
                                 std::fstream::out | std::ofstream::trunc);
    actual_file_out << std::setprecision(16) << calib_data;
    actual_file_out.close();
  }
}


/**
 * @brief Function to load calibration data from file
 * 
 */
void ArucoTF::loadCalibFromFile() {
  if (!ArucoTF::calib) {
    RCLCPP_INFO(this->get_logger(), "Saving calibration to file");

    std::string calib_path = ament_index_cpp::get_package_share_directory("lab8");
    calib_path += "/calibration/camera/logitech_extrinsics.json";

    // Open existing calibration
    std::ifstream calib_file_in;
    calib_file_in.open(calib_path);

    std::stringstream ss;
    if (calib_file_in.is_open()) {
      // Convert to string
      ss << calib_file_in.rdbuf();
    } else {
      std::cout << "Unable to open file" << std::endl;
    }
    calib_file_in.close();

    // Parse calibration text to json objectl
    nlohmann::json calib_data;
    try {
      calib_data = nlohmann::json::parse(ss);
    } catch (nlohmann::json::parse_error &e) {
      RCLCPP_WARN(this->get_logger(), "JSON Parse failed: %s", e.what());
    }

    // Find corresponding camera data in json
    std::string calib_section = "logitech_webcam";
    auto cam_section_itr = calib_data.find(calib_section);
    if (cam_section_itr != calib_data.end()) {
      std::cout << "FOUND " << calib_section << " in calibration file."
                << std::endl;
      // Get translation and rotation data from json
      std::vector<float> trans_json = calib_data[calib_section]["trans"];
      std::vector<float> rot_json = calib_data[calib_section]["rot"];

      // Vector in (x, y, z) format
      Eigen::Vector3f trans;
      trans(0) = trans_json[0];
      trans(1) = trans_json[1];
      trans(2) = trans_json[2];
      // Quaternion in (w,x,y,z) format
      Eigen::Quaternionf quat;
      quat.w() = rot_json[0];
      quat.x() = rot_json[1];
      quat.y() = rot_json[2];
      quat.z() = rot_json[3];

      std::cout << "Translation: " << trans_json << std::endl;
      std::cout << "Rotation: " << rot_json << std::endl;

      // Apply calibration data to camera
      tf2::Quaternion rot_quat_camToWorld(quat.x(), quat.y(), quat.z(), quat.w());
      rot_quat_camToWorld.normalize();

      // Get translation in tf2
      tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

      // Convert to tf2
      ArucoTF::tf_camToWorld.setRotation(rot_quat_camToWorld);
      ArucoTF::tf_camToWorld.setOrigin(trans_camToWorld);
      
      // Set calibrated
      ArucoTF::calib = true;
    } else {
      std::cout << "NOT FOUND " << calib_section << " in calibration file."
                << std::endl;
      ArucoTF::calib = false;
    }
  }
}


// ## Needs testing
/**
 * @brief Function to get transform from source to destination given 3D point
 * correspondences.
 * 
 */
void ArucoTF::estimateTransformPointToPoint() {
  // Compute transform
  Eigen::Matrix4f tf_srcToDst = Eigen::Matrix4f::Zero();
  RCLCPP_INFO(this->get_logger(), "Calibrating camera to world");
  tf_srcToDst = Eigen::umeyama(ArucoTF::samples_camToMarker,
                               ArucoTF::samples_markerToWorld);

  // Get rotation
  Eigen::Matrix3f rot = tf_srcToDst.topLeftCorner(3, 3);
  Eigen::Quaternionf rot_quat(rot);
  tf2::Quaternion rot_quat_camToWorld(rot_quat.x(), rot_quat.y(), rot_quat.z(),
                                      rot_quat.w());

  // Get translation
  Eigen::Vector3f trans = tf_srcToDst.topRightCorner(3, 1);
  tf2::Vector3 trans_camToWorld(trans(0), trans(1), trans(2));

  // Convert to tf2
  ArucoTF::tf_camToWorld.setRotation(rot_quat_camToWorld);
  ArucoTF::tf_camToWorld.setOrigin(trans_camToWorld);

  // Save data to file
  ArucoTF::saveCalibToFile(rot_quat, trans);

  // Set calibrated
  ArucoTF::calib = true;
}


// ## Needs testing
/**
 * @brief Function to get marker pose in camera coordinates
 * Sets class variable with returned value
 * 
 */
void ArucoTF::lookup_camToMarker() {
  RCLCPP_INFO(this->get_logger(), "Getting aruco transform");

  auto aruco_msg = aruco_opencv_msgs::msg::ArucoDetection();
  bool found = rclcpp::wait_for_message(aruco_msg, std::make_shared<ArucoTF>(), "aruco_detection", std::chrono::seconds(1));

  if (found){
    for (aruco_opencv_msgs::msg::MarkerPose marker : aruco_msg.markers) {
        if (marker.marker_id == aruco_calib_target) {
          ArucoTF::tform_camToMarker = marker.pose;
        }
      }
  }
}


// ## Needs testing
/**
 * @brief Function to get marker pose in camera coordinates
 * 
 * @param marker_id Overload marker_id
 * @return geometry_msgs::Transform - transform from target fram to source
 */
geometry_msgs::msg::Pose ArucoTF::lookup_camToMarker(const int &marker_id) {
  RCLCPP_INFO(this->get_logger(), "Getting aruco transform for Marker %i", marker_id);

  auto aruco_msg = aruco_opencv_msgs::msg::ArucoDetection();
  bool found = rclcpp::wait_for_message(aruco_msg, std::make_shared<ArucoTF>(), "aruco_detection", std::chrono::seconds(1));

  if (found){
    for (aruco_opencv_msgs::msg::MarkerPose marker : aruco_msg.markers) {
        if (marker.marker_id == aruco_calib_target) {
          ArucoTF::tform_camToMarker = marker.pose;
        }
      }
  }
}


// ## Need testing
/**
 * @brief Function to get transform from tool0 to world frame
 * The marker is placed at the tool0 location on the robot lookupTransform goes to target frame from source
 * 
 */
void ArucoTF::lookup_markerToWorld() {
  // TF2 listener for marker to world
  try {
    if (ArucoTF::tfBuffer->canTransform("base_link", "tool0", rclcpp::Time(0))) {
      RCLCPP_INFO(this->get_logger(), "Getting tool0 to world");
      ArucoTF::tform_markerToWorld =
          tfBuffer->lookupTransform("base_link", "tool0", rclcpp::Time(0));
    } else {
      ArucoTF::tform_markerToWorld = geometry_msgs::msg::TransformStamped();
      RCLCPP_INFO(this->get_logger(), "Could not find transform from world to tool0");
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    rclcpp::Rate(2.0).sleep();
  }
}


// ## Needs testing
/**
 * @brief Function to broadcast camera pose with respect to world
 */
void ArucoTF::broadcast_camToWorld() {
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Broadcasting camera to world");
  ArucoTF::tform_camToWorld.header.stamp = this->get_clock()->now();
  ArucoTF::tform_camToWorld.header.frame_id = "base_link";
  ArucoTF::tform_camToWorld.child_frame_id = "logitech_webcam";
  ArucoTF::tform_camToWorld.transform = tf2::toMsg(ArucoTF::tf_camToWorld);
  ArucoTF::br_camToWorld->sendTransform(ArucoTF::tform_camToWorld);
}


// ## Needs testing
/**
 * @brief Function to broadcast marker pose from camera frame to world frame
 */
void ArucoTF::broadcast_allMarkersToWorld() {

  auto aruco_msg = aruco_opencv_msgs::msg::ArucoDetection();
  bool found = rclcpp::wait_for_message(aruco_msg, std::make_shared<ArucoTF>(), "aruco_detection", std::chrono::seconds(1));

  // Lookup all markers
  if (found) {
    for (aruco_opencv_msgs::msg::MarkerPose marker : aruco_msg.markers) {
      // Check if marker id is in the list
      if (std::count(ArucoTF::aruco_track_targets.begin(), ArucoTF::aruco_track_targets.end(), marker.marker_id)) {
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10, "Broadcasting marker_" << marker.marker_id << " to world");

        geometry_msgs::msg::Pose tform_camToNewMarker = marker.pose;
        tf2::Transform tf_camToNewMarker;
        tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

        // Transform to world frame
        tf2::Transform tf_newMarkerToWorld =
            ArucoTF::tf_camToWorld * tf_camToNewMarker;

        // Convert back to geometry_msgs::TransformStamped
        geometry_msgs::msg::TransformStamped tform_newMarkerToWorld;
        tform_newMarkerToWorld.header.stamp = this->get_clock()->now();
        tform_newMarkerToWorld.header.frame_id = "base_link";
        tform_newMarkerToWorld.child_frame_id = "marker_" + std::to_string(marker.marker_id);
        tform_newMarkerToWorld.transform = tf2::toMsg(tf_newMarkerToWorld);

        ArucoTF::br_markersToWorld->sendTransform(tform_newMarkerToWorld);          
      }
    }
  }
}


// ## Needs testing
/**
 * @brief Function to compute marker pose from camera frame to world frame
 */
void ArucoTF::lookup_allMarkersToWorld(const int &marker_id,
                                       tf2::Transform &tf_newMarkerToWorld) {
  try {
    // Get marker_id to cam
    geometry_msgs::msg::Pose tform_camToNewMarker =
        ArucoTF::lookup_camToMarker(marker_id);
    tf2::Transform tf_camToNewMarker;
    tf2::fromMsg(tform_camToNewMarker, tf_camToNewMarker);

    // Transform to world frame
    tf_newMarkerToWorld = ArucoTF::tf_camToWorld * tf_camToNewMarker;
    tf_newMarkerToWorld.getRotation().normalize();

  } catch (const ArucoTF::NoTransformException &e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    rclcpp::Rate(2.0).sleep();
  }
}


// ## Needs testing
/**
 * @brief Function to take samples of camera to marker and marker to world poses from robot
 * 
 */
void ArucoTF::takeCalibrationSamples() {
  if (ArucoTF::calib) {
    RCLCPP_INFO(this->get_logger(), "Already calibrated, exiting.");
    return;
  };

  int sample_cnt = 0;
  RCLCPP_INFO(this->get_logger(), "Move robot to pose...");
  RCLCPP_INFO(this->get_logger(), "Press ENTER to record sample.");
  
  while (sample_cnt < num_samples) {
    RCLCPP_INFO(this->get_logger(), "Pose: %i/%i", sample_cnt + 1, ArucoTF::num_samples);
    getchar();

    ArucoTF::lookup_camToMarker();
    ArucoTF::samples_camToMarker.col(sample_cnt) =
        Eigen::Vector3f(ArucoTF::tform_camToMarker.position.x,
                        ArucoTF::tform_camToMarker.position.y,
                        ArucoTF::tform_camToMarker.position.z)
            .transpose();

    ArucoTF::lookup_markerToWorld();
    ArucoTF::samples_markerToWorld.col(sample_cnt) =
        Eigen::Vector3f(ArucoTF::tform_markerToWorld.transform.translation.x,
                        ArucoTF::tform_markerToWorld.transform.translation.y,
                        ArucoTF::tform_markerToWorld.transform.translation.z)
            .transpose();

    sample_cnt++;
  }
  RCLCPP_INFO(this->get_logger(), "Calibration samples gathered");
}


// ## Need testing
/**
 * @brief Compare calibration marker to ideal transformation
 *
 * @param marker_id
 */
void ArucoTF::verifyCalibration(const int &marker_id) {
  int sample_cnt = 0;
  RCLCPP_INFO_STREAM(this->get_logger(), "Move robot to pose...");
  RCLCPP_INFO_STREAM(this->get_logger(), "Press ENTER to record sample.");

  while (sample_cnt < ArucoTF::num_samples) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose: " << sample_cnt + 1 << "/"
                        << ArucoTF::num_samples);
    getchar();

    // Get marker to world using lookup_allMarkersToWorld()
    tf2::Transform tf_calibMarkerToWorld;
    ArucoTF::lookup_allMarkersToWorld(ArucoTF::aruco_calib_target,
                                      tf_calibMarkerToWorld);

    // Get tool0 TF using lookup_markerToWorld() function
    tf2::Stamped<tf2::Transform> tf_toolToWorld;
    ArucoTF::lookup_markerToWorld();
    tf2::fromMsg(ArucoTF::tform_markerToWorld, tf_toolToWorld);

    // Calculate the 7 dimensional error (x,y,z,qx,qy,qz,qw) between the two


    sample_cnt++;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "Verification samples gathered");

  // Once the errors are gathered, calculate sample mean vector and sample covariance matrix


}


/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char * argv[])
{
  // Initialise ROS
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(10.0);

  auto calibrate_cam = std::make_shared<ArucoTF>();

  RCLCPP_INFO(calibrate_cam->get_logger(), "------------------------------------------------------");
  
  const Eigen::Vector3f a(1, 2, 3);
  const Eigen::Quaternionf b(1, 2, 3, 4);

  // Testing Function
  calibrate_cam->loadCalibFromFile();


  // rclcpp::spin(calibrate_cam);
  // tf2::Transform tf_MarkerToWorld;
  // geometry_msgs::msg::Pose marker_pose;

  // Do while ok() 


  // Testing Eigen variables
  // std::cout << calibrate_cam->samples_camToMarker.rows() << " " << calibrate_cam->samples_camToMarker.cols() << "\n";
  // std::cout << calibrate_cam->samples_markerToWorld.rows() << " " << calibrate_cam->samples_markerToWorld.cols() << "\n";

  // calibrate_cam->samples_markerToWorld.col(0) = Eigen::Vector3f(1, 2, 3).transpose();
  // calibrate_cam->samples_camToMarker.col(0) = Eigen::Vector3f(4, 5, 3).transpose();

  // std::cout << calibrate_cam->samples_markerToWorld.col(0);
  // std::cout << calibrate_cam->samples_camToMarker.col(0);


  // This works for query one message
  // auto message = std_msgs::msg::String();
  // bool found = rclcpp::wait_for_message(message, std::make_shared<MinimalPublisher>(), "chatter", std::chrono::seconds(1));
  // RCLCPP_INFO(calibrate_cam->get_logger(), "scan found= %d", found);
  // RCLCPP_INFO(calibrate_cam->get_logger(), "Text found: %s", message.data.c_str());


  // End execution
  rclcpp::shutdown();
  return 0;
}
