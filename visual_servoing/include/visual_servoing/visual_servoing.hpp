

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
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

// Add all includes your project needs here
struct TransformHelper{
  Eigen::Vector3d position;
  Eigen::Matrix3d orientation;
};

struct ApriltagSave{
    double tag_id = 0.0;
    std::string label = "";
    std::string frame = "";
    std::string saved_transform = "";
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
   * @param robot_type
   * @param camera_type
   * @param tolerance_linear
   * @param tolerance_angular
   * @param topic_visual_servoing_on
   * @param topic_visual_servoing_save
   * @param topic_visual_servoing_clean
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
    RCLCPP_INFO(this->get_logger(), "robot_type : '%s'", robot_type);
    RCLCPP_INFO(this->get_logger(), "camera_type : '%s'", camera_type);
    RCLCPP_INFO(this->get_logger(), "tolerance_linear : '%lf'", tolerance_linear);
    RCLCPP_INFO(this->get_logger(), "tolerance_angular : '%lf'", tolerance_angular);
  }

  // Initialization
  void setupPublishers();
  void setupSubscribers();
  void getParameters();
  void initVariables();

  // Callback to receive Twist commands from the teleop node
  void visualServoingOnCallback(const std_msgs::msg::Bool msg);
  void visualServoingSaveCallback(const std_msgs::msg::String msg);
  void visualServoingCleanCallback(const std_msgs::msg::String msg);
  void tagCallback(const extender_msgs::msg::SharedControlGoalArray msg);

  void timer_callback();

  void readYamlApriltags();
  bool writeYamlApriltags(
    std::string yaml_path,
    double tag_id,
    const std::string label,
    const Eigen::Vector3d position,
    const Eigen::Quaterniond orientation);
  bool cleanYamlApriltags(
    std::string yaml_path,
    double tag_id,
    std::list<ApriltagSave> listApriltagSave);
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
  std_msgs::msg::String latest_visual_servoing_clean;
  extender_msgs::msg::SharedControlGoalArray latest_tag_detected;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr visual_servoing_velocity_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr visual_servoing_error_pub;

  rclcpp::Subscription<extender_msgs::msg::SharedControlGoalArray>::SharedPtr apriltag_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visual_servoing_on_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_servoing_save_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_servoing_clean_sub;

  // Input Param
  std::string robot_type;     // param for yaml path file
  std::string camera_type;    // param for yaml path file
  double camera_number;
  double lambda;              // param #1 - gain
  double tolerance_linear;    // param - tolerance for linear error
  double tolerance_angular;   // param - tolerance for angular error
  std::string topic_visual_servoing_on;
  std::string topic_visual_servoing_save;
  std::string topic_visual_servoing_clean;
  TransformHelper EEtoCAM;      // param #2 - calibration saved on yaml file
  TransformHelper CAMtoTAGd;  // param #3 - Apriltag saved on yaml file
  Eigen::Vector3d t_CAMtoTAGd;
  Eigen::Quaterniond r_CAMtoTAGd;
  TransformHelper CAMtoTAG;   // param #4 - streaming
  TransformHelper BtoEE;      // param #5 - streaming

  // Subscriber
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // read saving apriltags position in Yaml
  std::string yaml_path_saved_tag_goals;
  std::string yaml_path_transform_EEtoCAM;
  std::string ee_frame, camera_frame;
  ApriltagSave apriltagSave;
  ApriltagSave apriltag;
  std::list<ApriltagSave> listApriltagSave;
  int number_of_position_saved;
  int position_saved_step;
  int position_current_step;

  // get qontrol_controller v_max param
  double command_max_linear_velocity_;
  double command_max_angular_velocity_;

};