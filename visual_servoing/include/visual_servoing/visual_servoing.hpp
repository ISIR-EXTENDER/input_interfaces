

// This includes are mandatory
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"

// 
#include "extender_msgs/msg/shared_control_goal.hpp"
#include "extender_msgs/msg/shared_control_goal_array.hpp"

// for YAML
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>

// Add all includes your project needs here
struct TransformHelper{
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
};

struct ApriltagSave{
    double tag_id = 0.0;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

class VisualServoing : public rclcpp::Node
{
public:
  VisualServoing();

private:
  /// -------------------------------------------------------------------- Functions
  /**
   * @brief Template function to declare and get parameters with default values.
   * @tparam T Type of the parameter.
   * @param lambda Parameter name.
   */
  template <typename T>
  
  void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
  {
    if (!this->has_parameter(name))
    {
      this->declare_parameter(name, default_value);
    }
    variable = this->get_parameter(name).get_value<T>();
    RCLCPP_INFO(this->get_logger(), "lambda : '%lf'", lambda);
  }

  // Initialization
  void setupPublishers();
  void setupSubscribers();
  void getParameters();

  // Callback to receive Twist commands from the teleop node
  void visualServoingOnCallback(const std_msgs::msg::Bool msg);
  void visualServoingSaveCallback(const std_msgs::msg::String msg);
  void tagCallback(const extender_msgs::msg::SharedControlGoalArray msg);

  void timer_callback();

  void readYamlApriltags(double tag_id_to_follow);
  bool writeYamlApriltags(
    std::string yaml_path,
    double tag_id,
    const std::string label,
    const Eigen::Vector3d position,
    const Eigen::Quaterniond orientation);
  void readYamlTransformEEtoCAM();

  void sat (
    Eigen::Vector3d& v_c,
    Eigen::Vector3d& omega_c,
    float v_max_max,
    float omega_max_max);

  Eigen::Vector3d quatToThetaU(Eigen::Quaterniond quaternion);

  ///------------------------------------------------- Variables
  std_msgs::msg::Bool latest_visual_servoing_on;
  std_msgs::msg::String latest_visual_servoing_save;
  extender_msgs::msg::SharedControlGoalArray latest_tag_detected;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr visual_servoing_velocity_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr visual_servoing_error_pub;

  rclcpp::Subscription<extender_msgs::msg::SharedControlGoalArray>::SharedPtr apriltag_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_servoing_on_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_servoing_save_sub;

  // Input Param
  double lambda;              // param #1 - gain
  TransformHelper EEtoCAM;      // param #2 - calibration saved on yaml file
  TransformHelper CAMtoTAGd;  // param #3 - Apriltag saved on yaml file
  TransformHelper CAMtoTAG;   // param #4 - streaming
  TransformHelper BtoEE;      // param #5 - streaming

  // Subscriber
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // read saving apriltags position in Yaml
  std::string yaml_path;
  std::string yaml_path_transform_EEtoCAM;
  ApriltagSave apriltagSave;
  ApriltagSave apriltag;

};