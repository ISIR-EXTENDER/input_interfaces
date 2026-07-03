#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "visual_servoing/visual_servoing.hpp"
#include "extender_msgs/msg/shared_control_goal.hpp"
#include "extender_msgs/msg/shared_control_goal_array.hpp"

VisualServoing::VisualServoing()
    : Node("visual_servoing")
{
    using namespace std::chrono_literals;

    getParameters();
    setupPublishers();
    setupSubscribers();
    readYamlTransformEEtoCAM();
    timer_ = this->create_wall_timer(33ms, std::bind(&VisualServoing::timer_callback, this));      // 1/30Hz = 0.033s => 33ms
}

void VisualServoing::getParameters()
{
    std::cout << "getParameters             [*    ] " << std::endl;
    
    // Read YAML parameters file 
    this->declare_parameter("lambda", 0.0);
    lambda = this->get_parameter("lambda").as_double();
    
    // read saving apriltags position in Yaml
    declare_parameter<std::string>("yaml_path", "/home/robingibaud/ros2_ws/src/extender_workspace/src/visual_servoing/config/saved_tag_goals.yaml");
    yaml_path = get_parameter("yaml_path").as_string();
    declare_parameter<std::string>("yaml_path_transform_EEtoCAM", "/home/robingibaud/ros2_ws/src/extender_workspace/src/visual_servoing/config/handeye_tf_kinovaCam.yaml");
    yaml_path_transform_EEtoCAM = get_parameter("yaml_path_transform_EEtoCAM").as_string();
}

void VisualServoing::setupPublishers()
{
    std::cout << "setupPublishers           [**   ] " << std::endl;
    visual_servoing_velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/visual_servoing/velocity_command", 1);
    visual_servoing_error_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/visual_servoing/error_TAGtoTAGd", 1);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

void VisualServoing::setupSubscribers()
{
    std::cout << "setupSubscribers          [***  ] " << std::endl;
    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    
    apriltag_sub = this->create_subscription<extender_msgs::msg::SharedControlGoalArray>(
        "/tag_detections", 1,
        std::bind(&VisualServoing::tagCallback, this, std::placeholders::_1));
    visual_servoing_on_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/ui/visual_servoing/on", 1,
        std::bind(&VisualServoing::visualServoingOnCallback, this, std::placeholders::_1));
    visual_servoing_save_sub = this->create_subscription<std_msgs::msg::String>(
        "/ui/visual_servoing/save", 1,
        std::bind(&VisualServoing::visualServoingSaveCallback, this, std::placeholders::_1));
    
}

// -------------------------------------------------------------------------
// Interface UI : On button callback
// -------------------------------------------------------------------------

void VisualServoing::visualServoingOnCallback(const std_msgs::msg::Bool msg)
{
    latest_visual_servoing_on = msg;
    //RCLCPP_INFO(this->get_logger(), "latest_visual_servoing_on : '%d'", latest_visual_servoing_on);
}

// -------------------------------------------------------------------------
// Interface UI : Save button callback
// -------------------------------------------------------------------------
void VisualServoing::visualServoingSaveCallback(const std_msgs::msg::String msg)
{
    latest_visual_servoing_save = msg;
    std::string label = "";
    if (writeYamlApriltags(yaml_path, apriltag.tag_id, label, apriltag.position, apriltag.orientation) == true)
    {
        RCLCPP_INFO(this->get_logger(), "tag '%lf' saved in yaml file", apriltag.tag_id);
    };
    //RCLCPP_INFO(this->get_logger(), "latest_visual_servoing_save : '%s'", latest_visual_servoing_save.data);
}


void VisualServoing::tagCallback(const extender_msgs::msg::SharedControlGoalArray msg)
{
    latest_tag_detected = msg;
    //RCLCPP_INFO(this->get_logger(), "tagCallback : message processing ...");

    if (msg.goal_array.empty()){

        //RCLCPP_INFO(this->get_logger(), "tagCallback : empty message ...");
        apriltag.position[0]=0.0;
        apriltag.position[1]=0.0;
        apriltag.position[2]=0.0;
        apriltag.orientation.w()=0.0;
        apriltag.orientation.x()=0.0;
        apriltag.orientation.y()=0.0;
        apriltag.orientation.z()=0.0;
    }
    else {        
        apriltag.tag_id = msg.goal_array[0].id;

        Eigen::Vector3d temp_t_apriltag;
        temp_t_apriltag <<
            msg.goal_array[0].goal_pose.position.x,
            msg.goal_array[0].goal_pose.position.y,
            msg.goal_array[0].goal_pose.position.z;
        
        Eigen::Quaterniond temp_r_apriltag(
            msg.goal_array[0].goal_pose.orientation.w,
            msg.goal_array[0].goal_pose.orientation.x,
            msg.goal_array[0].goal_pose.orientation.y,
            msg.goal_array[0].goal_pose.orientation.z);
        
        apriltag.position = temp_t_apriltag;
        apriltag.orientation = temp_r_apriltag;
    }

}

void VisualServoing::readYamlApriltags(double tag_id_to_follow)                                    // R.G
{
    //std::cout << "readYamlApriltags         [**** ] " << std::endl;
    // Read Yaml
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "failed to open " << yaml_path << std::endl;
    }
    
    cv::FileNode tags = fs.root();

    for (auto it = tags.begin(); it != tags.end(); ++it)
    {
        int tag_id;
        std::string label, frame, saved_transform;
        
        (*it)["tag_id"] >> tag_id;
        (*it)["label"] >> label;
        (*it)["frame"] >> frame;
        (*it)["saved_transform"] >> saved_transform;

        //std::cout << "tag_id -> " << tag_id << std::endl;
        
        std::vector<double> position;
        std::vector<double> orientation;

        (*it)["position"] >> position;
        (*it)["orientation_wxyz"] >> orientation;
        
        if (tag_id == tag_id_to_follow){
            apriltagSave.tag_id = tag_id;
            apriltagSave.position <<
                position[0],
                position[1],
                position[2];
            Eigen::Quaterniond temp_local_save (
                orientation[0],
                orientation[1],
                orientation[2],
                orientation[3]);
            apriltagSave.orientation = temp_local_save;
        }
        //std::cout << "local_save.position[0] = " << local_save.position[0] << std::endl;
        //RCLCPP_INFO(this->get_logger(), "param #3 - apriltagSave = '%lf' ['%lf','%lf','%lf'] ['%lf','%lf','%lf','%lf']", apriltagSave.tag_id, apriltagSave.position[0], apriltagSave.position[1], apriltagSave.position[2], apriltagSave.orientation.w(), apriltagSave.orientation.x(), apriltagSave.orientation.y(), apriltagSave.orientation.z());
    }

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera_link";
    t.child_frame_id = "saved_tag";

    t.transform.translation.x = apriltagSave.position[0];
    t.transform.translation.y = apriltagSave.position[1];
    t.transform.translation.z = apriltagSave.position[2];
    t.transform.rotation.x = apriltagSave.orientation.x();
    t.transform.rotation.y = apriltagSave.orientation.y();
    t.transform.rotation.z = apriltagSave.orientation.z();
    t.transform.rotation.w = apriltagSave.orientation.w();

    tf_static_broadcaster_->sendTransform(t);

    fs.release();
}

bool VisualServoing::writeYamlApriltags(
    std::string yaml_path,
    double tag_id,
    const std::string label,
    const Eigen::Vector3d position,
    const Eigen::Quaterniond orientation)
{
    std::ofstream file(yaml_path, std::ios::app);

    if (!file.is_open())
    {
        //RCLCPP_ERROR(get_logger(), "Failed to open YAML file: %s", yaml_path.c_str());
        return false;
    }

    file << "- tag_id: " << tag_id << "\n";
    file << "  label: \"" << label << "\"\n";
    file << "  frame: \"tag_" << tag_id << "\"\n";
    file << "  saved_transform: \"tag_T_ee\"\n";
    file << "  position: ["
            << position.x() << ", "
            << position.y() << ", "
            << position.z() << "]\n";
    file << "  orientation_wxyz: ["
            << orientation.w() << ", "
            << orientation.x() << ", "
            << orientation.y() << ", "
            << orientation.z() << "]\n";
    file.close();
    return true;
}

void VisualServoing::readYamlTransformEEtoCAM()                                    // R.G
{
    std::cout << "readYamlTransformEEtoCAM  [*****] " << std::endl;
    // Read Yaml
    cv::FileStorage fs(yaml_path_transform_EEtoCAM, cv::FileStorage::READ);

    // Info about Yaml file
    //std::cout << "empty: " << root.empty() << std::endl;
    //std::cout << "isMap: " << root.isMap() << std::endl;
    //std::cout << "isSeq: " << root.isSeq() << std::endl;

    if (!fs.isOpened())
    {
        std::cerr << "failed to open " << yaml_path_transform_EEtoCAM << std::endl;
        return;
    }
        
    double tx, ty, tz;
    double qw, qx, qy, qz;
    
    fs["tx"] >> tx;
    fs["ty"] >> ty;
    fs["tz"] >> tz;
    fs["qw"] >> qw;
    fs["qx"] >> qx;
    fs["qy"] >> qy;
    fs["qz"] >> qz;
        
    Eigen::Vector3d t_EEtoCAM;
    t_EEtoCAM << tx, ty, tz;
    Eigen::Quaterniond r_EEtoCAM(qw, qx, qy, qz);
    EEtoCAM.position = t_EEtoCAM;
    EEtoCAM.orientation = r_EEtoCAM.toRotationMatrix();

    
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "end_effector_link";
    t.child_frame_id = "camera_link";

    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.translation.z = tz;
    t.transform.rotation.x = qx;
    t.transform.rotation.y = qy;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;

    tf_static_broadcaster_->sendTransform(t);

    fs.release();
}

Eigen::Vector3d VisualServoing::quatToThetaU(                                  //R.G
    Eigen::Quaterniond quaternion
)
{
    // Convertion the error qDiff (XYZW) to error Finished rotation (XYZ)
    Eigen::Vector3d theta_u;
    theta_u <<  0.0,0.0,0.0;
    Eigen::Vector3d b_s;
    b_s <<  0.0,0.0,0.0;
    if (quaternion.w()>0.9999999999999){
        b_s[0] = 0.0;      // error RX
        b_s[1] = 0.0;      // error RY
        b_s[2] = 0.0;      // error RZ
    }
    else{
        float s;
        s = quaternion.w();
        float local_theta;
        local_theta = 2 * std::acos(s);
        if (local_theta > M_PI){    // in case of theta is over 180°,
            local_theta -= 2*M_PI;    // theta = theta - 180° to stay between 0 and +180°
        }
        float sigma;
        sigma = std::sqrt(1 - s*s);
        b_s[0] = -(local_theta / sigma) * quaternion.x(); // error RX
        b_s[1] = -(local_theta / sigma) * quaternion.y(); // error RY
        b_s[2] = -(local_theta / sigma) * quaternion.z(); // error RZ
    }
    // result  => theta_u
    theta_u[0]=b_s[0];
    theta_u[1]=b_s[1];
    theta_u[2]=b_s[2];
    //print("theta.u = {} {}  {}".format( (1/ np.sin(local_theta/2)) * qDiff[0], (1/ np.sin(local_theta/2)) * qDiff[1], (1 / np.sin(local_theta/2)) * qDiff[2]))
    
    return theta_u;
}

// SECURITY : Saturation des vitesse
void VisualServoing::sat (
    Eigen::Vector3d& v_c,
    Eigen::Vector3d& omega_c,
    float v_max_max,
    float omega_max_max)
{
    float norme_linear = 0.0;
    float norme_angular = 0.0;

    norme_linear = float(std::sqrt(v_c[0]*v_c[0] + v_c[1]*v_c[1] + v_c[2]*v_c[2]));
    if (norme_linear > v_max_max){
        for (int i = 0; i < 3; i++) {
            v_c[i] = v_c[i]*(v_max_max/norme_linear);
            omega_c[i] = omega_c[i]*(v_max_max/norme_linear);
        }
    }

    norme_angular = float(std::sqrt(omega_c[0]*omega_c[0] + omega_c[1]*omega_c[1] + omega_c[2]*omega_c[2]));
    if (norme_linear > v_max_max){
        for (int i = 0; i < 3; i++) {
            v_c[i] = v_c[i] * (omega_max_max/norme_angular);
            omega_c[i] = omega_c[i] * (omega_max_max/norme_angular);
        }
    }
}

void VisualServoing::timer_callback(){
    if (latest_visual_servoing_on.data == false){
        return;
    }
    //std::cout << "Visual_servoing : on" << std::endl;
    double sum_position_apriltag_callback;
    sum_position_apriltag_callback = apriltag.position[0] + apriltag.position[1] + apriltag.position[2];
    if (sum_position_apriltag_callback == 0.0){
        //std::cout << "no apriltag -> skip visual servoing computation" << std::endl;
        geometry_msgs::msg::TwistStamped vel_to_pub;
        vel_to_pub.header.stamp = this->now();
        vel_to_pub.twist.linear.x = 0.0;
        vel_to_pub.twist.linear.y = 0.0;
        vel_to_pub.twist.linear.z = 0.0;
        vel_to_pub.twist.angular.x = 0.0;
        vel_to_pub.twist.angular.y = 0.0;
        vel_to_pub.twist.angular.z = 0.0;
        visual_servoing_velocity_pub->publish(vel_to_pub);
    }
    
    else{
        // param #1 - gain                                                                                          // OK
        //RCLCPP_INFO(this->get_logger(), "param #1 - lambda = '%lf'", lambda);

        // param #2 - transformation of EE's frame to Camera's frame                                                // OK
        // already done in "readYamlTransformEEtoCAM" function
        //RCLCPP_INFO(this->get_logger(), "param #2 - EEtoCAM = ['%lf','%lf','%lf']", EEtoCAM.position.x(), EEtoCAM.position.y(), EEtoCAM.position.z());

        // param #3 - transformation of Camera's frame to Tag frame saved                                           // OK
        readYamlApriltags(apriltag.tag_id);
        Eigen::Vector3d t_CAMtoTAGd;
        t_CAMtoTAGd = apriltagSave.position;
        Eigen::Quaterniond r_CAMtoTAGd;
        r_CAMtoTAGd = apriltagSave.orientation;
        CAMtoTAGd.position = t_CAMtoTAGd;
        CAMtoTAGd.orientation = r_CAMtoTAGd.toRotationMatrix();
        //RCLCPP_INFO(this->get_logger(), "param #3 - CAMtoTAGd = '%lf' ['%lf','%lf','%lf'] ['%lf','%lf','%lf','%lf']", apriltagSave.tag_id, t_CAMtoTAGd[0], t_CAMtoTAGd[1], t_CAMtoTAGd[2], r_CAMtoTAGd.w(), r_CAMtoTAGd.x(), r_CAMtoTAGd.y(), r_CAMtoTAGd.z());

        // param #4 - transformation of Camera's frame to Tag frame currently read                                  // OK
        // autre option : passer par le "topic /tag_detector"
        Eigen::Vector3d t_CAMtoTAG;
        t_CAMtoTAG = apriltag.position;
        Eigen::Quaterniond r_CAMtoTAG;
        r_CAMtoTAG = apriltag.orientation;
        CAMtoTAG.position = t_CAMtoTAG;
        CAMtoTAG.orientation = r_CAMtoTAG.toRotationMatrix();
        //RCLCPP_INFO(this->get_logger(), "param #4 - apriltag = '%lf' ['%lf','%lf','%lf'] ['%lf','%lf','%lf','%lf']", apriltag.tag_id, apriltag.position[0], apriltag.position[1], apriltag.position[2], apriltag.orientation.w(), apriltag.orientation.x(), apriltag.orientation.y(), apriltag.orientation.z());

        CAMtoTAG.position = t_CAMtoTAG;
        CAMtoTAG.orientation = r_CAMtoTAG.toRotationMatrix();
        
        // param #5 - transformation of robot Base's frame to EE's frame currently read                             // OK
        auto temp_BtoEE = tf_buffer_->lookupTransform("base_link", "end_effector_link",tf2::TimePointZero);
        Eigen::Vector3d t_BtoEE;
        t_BtoEE <<  temp_BtoEE.transform.translation.x,
                    temp_BtoEE.transform.translation.y,
                    temp_BtoEE.transform.translation.z;
        Eigen::Quaterniond r_BtoEE(
                    temp_BtoEE.transform.rotation.w,
                    temp_BtoEE.transform.rotation.x,
                    temp_BtoEE.transform.rotation.y,
                    temp_BtoEE.transform.rotation.z);
        BtoEE.position = t_BtoEE;
        BtoEE.orientation = r_BtoEE.toRotationMatrix();
        //RCLCPP_INFO(this->get_logger(), "param #5 - BtoEE : ['%lf','%lf','%lf'] ['%lf','%lf','%lf','%lf']", t_BtoEE[0], t_BtoEE[1], t_BtoEE[2], r_BtoEE.w(), r_BtoEE.x(), r_BtoEE.y(), r_BtoEE.z());
        
        // 1 - Computation velocity of tagEtag_in_cam ************************************************************************************************************************
        Eigen::Vector3d velocity_of_tagEtag_in_cam;
        velocity_of_tagEtag_in_cam = lambda * (CAMtoTAGd.position - CAMtoTAG.position);
        
        TransformHelper TAGtoCAM;
        TAGtoCAM.orientation = CAMtoTAG.orientation.transpose();
        TransformHelper TAGtoTAGd;
        TAGtoTAGd.orientation = TAGtoCAM.orientation*CAMtoTAGd.orientation;

        Eigen::AngleAxis< double > r_TAGtoTAGd_aa (TAGtoTAGd.orientation);
        Eigen::Vector3d TAG_theta_u = r_TAGtoTAGd_aa.angle() * r_TAGtoTAGd_aa.axis();
        //std::cout << "TAG_theta_u = ["<< TAG_theta_u[0] << " , " << TAG_theta_u[1] << " , " << TAG_theta_u[2] << " ]" << r_TAGtoTAGd_aa.angle() << std::endl;
        
        Eigen::Vector3d CAM_theta_u;
        CAM_theta_u = CAMtoTAG.orientation * TAG_theta_u;
        Eigen::Vector3d omega_of_tagEtag_in_cam;
        omega_of_tagEtag_in_cam = lambda * CAM_theta_u;
        std::cout << "CAM_theta_u = ["<< CAM_theta_u[0] << " , " << CAM_theta_u[1] << " , " << CAM_theta_u[2] << " ]" << std::endl;
        //std::cout << "omega_of_tagEtag_in_cam = ["<< omega_of_tagEtag_in_cam[0] << " , " << omega_of_tagEtag_in_cam[1] << " , " << omega_of_tagEtag_in_cam[2] << " ]" << std::endl;

        // 2 - Computation velocity of tagEee_in_b ************************************************************************************************************************
        Eigen::Vector3d velocity_of_tagEcam_in_tag = -velocity_of_tagEtag_in_cam;
        Eigen::Vector3d velocity_of_tagEee_in_b = velocity_of_tagEcam_in_tag;
        
        // 3 - Computation velocity of eeEee_in_b************************************************************************************************************************
        Eigen::Vector3d velocity_of_ee_in_cam;
        velocity_of_ee_in_cam = velocity_of_tagEee_in_b+((EEtoCAM.orientation*EEtoCAM.position)+CAMtoTAG.position).cross(-omega_of_tagEtag_in_cam);
        
        Eigen::Vector3d velocity_of_ee_in_b;
        Eigen::Vector3d omega_of_ee_in_b;
        velocity_of_ee_in_b = (BtoEE.orientation*EEtoCAM.orientation)*velocity_of_ee_in_cam;
        //omega_of_ee_in_b = (BtoEE.orientation*EEtoCAM.orientation)*omega_of_tagEtag_in_cam;
        omega_of_ee_in_b = omega_of_tagEtag_in_cam;
        
        // Saturation
        
        float v_max_max = 0.2;
        float characteristic_lenght = 0.5;
        float omega_max_max = v_max_max/characteristic_lenght ;
        sat(velocity_of_ee_in_b, omega_of_ee_in_b, v_max_max,omega_max_max);
        

        // publication
        //  outputs : 
        //      -> velocity_of_ee_in_b[3]
        //      -> omega_of_ee_in_b[3]
        geometry_msgs::msg::TwistStamped vel_to_pub;
        vel_to_pub.header.stamp = this->now();

        vel_to_pub.twist.linear.x = velocity_of_ee_in_b[0];
        vel_to_pub.twist.linear.y = velocity_of_ee_in_b[1];
        vel_to_pub.twist.linear.z = velocity_of_ee_in_b[2];
        vel_to_pub.twist.angular.x = omega_of_ee_in_b[0];
        vel_to_pub.twist.angular.y = omega_of_ee_in_b[1];
        vel_to_pub.twist.angular.z = omega_of_ee_in_b[2];
        
        /*
        vel_to_pub.twist.linear.x = 0.0;
        vel_to_pub.twist.linear.y = 0.0;
        vel_to_pub.twist.linear.z = 0.0;
        vel_to_pub.twist.angular.x = 0.0;
        vel_to_pub.twist.angular.y = 0.0;
        vel_to_pub.twist.angular.z = 0.0;
        */

        visual_servoing_velocity_pub->publish(vel_to_pub);

        // Débug
        geometry_msgs::msg::TwistStamped error_pub;
        error_pub.header.stamp = this->now();
        error_pub.twist.linear.x = abs(velocity_of_tagEtag_in_cam[0]/lambda);
        error_pub.twist.linear.y = abs(velocity_of_tagEtag_in_cam[1]/lambda);
        error_pub.twist.linear.z = abs(velocity_of_tagEtag_in_cam[2]/lambda);
        error_pub.twist.angular.x = abs(CAM_theta_u[0]);
        error_pub.twist.angular.y = abs(CAM_theta_u[1]);
        error_pub.twist.angular.z = abs(CAM_theta_u[2]);
        visual_servoing_error_pub->publish(error_pub);

        //std::cout << "vel_to_pub = [" << vel_to_pub.twist.linear.x << " , " << vel_to_pub.twist.linear.y << " , " << vel_to_pub.twist.linear.x << " ]" << std::endl;
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualServoing>());
  rclcpp::shutdown();
  return 0;
}