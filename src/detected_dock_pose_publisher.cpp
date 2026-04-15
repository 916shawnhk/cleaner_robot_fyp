/**
 * @file detected_dock_pose_publisher.cpp
 * @brief Publishes the pose of an AprilTag located on/near a docking station.
 *
 * Listens to TF2 for the transform between the camera optical frame and the
 * detected AprilTag frame, then republishes it as a geometry_msgs/PoseStamped
 * on /detected_dock_pose for use by the Nav2 docking server.
 *
 * Subscription Topics:
 *     /tf (tf2_msgs/TFMessage): Transform tree
 *
 * Publishing Topics:
 *     /detected_dock_pose (geometry_msgs/PoseStamped): AprilTag pose relative
 *                                                      to the camera optical frame
 *
 * Parameters:
 *     parent_frame (string, default: "camera_link_optical"): Camera optical frame
 *     child_frame  (string, default: "tag36h11:0"):          AprilTag TF frame
 *     publish_rate (double, default: 10.0):                  Publishing rate in Hz
 *
 * Adapted from yahboom_rosmaster_docking for cleaner_robot_fyp.
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class DetectedDockPosePublisher : public rclcpp::Node
{
public:
  DetectedDockPosePublisher()
  : Node("detected_dock_pose_publisher")
  {
    // Parameters — defaults match cleaner_robot_fyp camera frame names
    this->declare_parameter("parent_frame", "camera_link_optical");
    this->declare_parameter("child_frame",  "tag36h11:0");
    this->declare_parameter("publish_rate", 10.0);

    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_  = this->get_parameter("child_frame").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    dock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&DetectedDockPosePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "DetectedDockPosePublisher started: parent='%s' child='%s'",
      parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.stamp    = this->get_clock()->now();
    dock_pose.header.frame_id = parent_frame_;

    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        parent_frame_,
        child_frame_,
        tf2::TimePointZero);

      dock_pose.pose.position.x  = transform.transform.translation.x;
      dock_pose.pose.position.y  = transform.transform.translation.y;
      dock_pose.pose.position.z  = transform.transform.translation.z;
      dock_pose.pose.orientation = transform.transform.rotation;

      dock_pose_pub_->publish(dock_pose);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  std::string parent_frame_;
  std::string child_frame_;

  std::unique_ptr<tf2_ros::Buffer>              tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>   tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::TimerBase::SharedPtr                  timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectedDockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
