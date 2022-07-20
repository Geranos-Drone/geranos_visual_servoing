#include <geranos_visual_servoing/visual_servoing_node.h>

namespace geranos {

  void getPointMsgFromEigen(const Eigen::Vector3d& vec, geometry_msgs::PointStamped* msg) {
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";
    msg->point.x = vec(0);
    msg->point.y = vec(1);
    msg->point.z = vec(2);
  }

  VisualServoingNode::VisualServoingNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) : 
    nh_(nh),
    private_nh_(private_nh),
    tf2_(buffer_),
    received_odometry_(false),
    received_pole_pose_(false),
    activated_(false) {
      initializeSubscribers();
      initializePublishers();
      initializeServices();
      loadParams();
      loadTFs();  
      timer_run_ = nh_.createTimer(ros::Duration(1.0/200.0), &VisualServoingNode::run, this);
      states_.clear();
    }

  VisualServoingNode::~VisualServoingNode() {}

  void VisualServoingNode::initializeSubscribers() {
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &VisualServoingNode::odometryCallback, this);
    pole_vicon_sub_ = nh_.subscribe("geranos_pole_white/vrpn_client/estimated_transform", 1, &VisualServoingNode::poleViconCallback, this);
    pose_estimate_sub_ = nh_.subscribe("PolePoseNode/EstimatedPose", 1, &VisualServoingNode::poseEstimateCallback, this);    
  }

  void VisualServoingNode::initializePublishers() {
    // create publisher for trajectory
    pub_trajectory_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 0);
    // create publisher for estimated pole position
    pole_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>(ros::this_node::getName() + "/estimated_pole_position", 0);
    // create publisher for RVIZ markers
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/trajectory_markers", 0);
    // create publisher for estimation error
    error_pub_ = nh_.advertise<geometry_msgs::PointStamped>(ros::this_node::getName() + "/error_vector", 0);
    // create publisher for transformed odom
    transformed_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(ros::this_node::getName() + "/transformed_odometry", 1);    
  }

  void VisualServoingNode::initializeServices() {
    activate_service_ = nh_.advertiseService("activate_servoing_service", &VisualServoingNode::activateServoingSrv, this);
    grab_pole_service_ = nh_.advertiseService("grab_pole_service", &VisualServoingNode::grabPoleSrv, this);
    lift_pole_service_ = nh_.advertiseService("lift_pole_service", &VisualServoingNode::liftPoleSrv, this);
  }

  void VisualServoingNode::odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("VisualServoingNode received first odometry!");
    mav_msgs::EigenOdometry odom;
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odom);
    transformOdometry(odom);
    publishOdometry(odom);
    current_odometry_ = odom;
    received_odometry_ = true;
  }
  void VisualServoingNode::publishOdometry(const mav_msgs::EigenOdometry& odom) {
    nav_msgs::Odometry odometry_msg;
    mav_msgs::msgOdometryFromEigen(odom, &odometry_msg);
    transformed_odom_pub_.publish(odometry_msg);
  }

  void VisualServoingNode::transformOdometry(mav_msgs::EigenOdometry& odometry) {
    Eigen::Matrix3d R_B_imu = T_B_imu_.rotation();  // rotation from imu to body frame
    Eigen::Vector3d r_B_imu_B = T_B_imu_.translation();  // body to imu offset expressed in base frame
    Eigen::Matrix3d R_W_B = odometry.orientation_W_B.toRotationMatrix();
    // add translational offset between imu and body frame
    odometry.position_W -= R_W_B * R_B_imu * r_B_imu_B;

    // transform velocity 
    Eigen::Vector3d v_rot = odometry.angular_velocity_B.cross(r_B_imu_B);
    // rotate velocity vector into body frame and add rotational contribution
    odometry.velocity_B = R_B_imu * odometry.velocity_B - v_rot;
  }

  void VisualServoingNode::poseEstimateCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // transform pose to base frame
    Eigen::Vector3d pole_pos_C = mav_msgs::vector3FromPointMsg(pose_msg->pose.position);
    Eigen::Vector3d pole_pos_B = t_B_cam_ + R_B_cam_ * pole_pos_C;
    current_pole_pos_B_ = pole_pos_B;

    // transform position to world frame
    Eigen::Matrix3d R_W_B = current_odometry_.orientation_W_B.toRotationMatrix();
    current_pole_pos_ = current_odometry_.position_W + R_W_B * pole_pos_B;
    received_pole_pose_ = true;

    // publish pole position
    geometry_msgs::PointStamped pole_pos_msg;
    getPointMsgFromEigen(current_pole_pos_, &pole_pos_msg);
    pole_pos_pub_.publish(pole_pos_msg);
  }

  void VisualServoingNode::poleViconCallback(const geometry_msgs::TransformStamped& pole_transform_msg) {
    ROS_INFO_ONCE("[VisualServoingNode] Received first transform of Pole!");
    mav_msgs::eigenTrajectoryPointFromTransformMsg(pole_transform_msg, &pole_trajectory_point_);
    current_pole_pos_vicon_ = pole_trajectory_point_.position_W;
    
    //calculate error from estimation
    if (received_pole_pose_) {
      Eigen::Vector3d error_vector = current_pole_pos_vicon_ - current_pole_pos_;
      geometry_msgs::PointStamped error_msg;
      getPointMsgFromEigen(error_vector, &error_msg);
      error_pub_.publish(error_msg);
    }
  }

  bool VisualServoingNode::grabPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    if (activated_) {
      activated_ = false;
    }
    Eigen::Vector3d waypoint_position = current_odometry_.position_W - Eigen::Vector3d(0.0, 0.0, 1.1);
    Eigen::Quaterniond waypoint_orientation = mav_msgs::quaternionFromYaw(start_yaw_);
    Eigen::Vector3d velocity_command (0.0, 0.0, 0.0);
    Eigen::Vector3d ang_velocity_command (0.0, 0.0, 0.0);
    double duration = 5e9;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    generateTrajectoryMsg(trajectory_msg, waypoint_position, 
                          waypoint_orientation, velocity_command, 
                          ang_velocity_command, duration);
    pub_trajectory_.publish(trajectory_msg);

    // get marker to display Waypoint in RVIZ
    visualization_msgs::MarkerArray markers;
    double marker_distance = 0.02; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mav_trajectory_generation::drawMavSampledTrajectory(states_, marker_distance, frame_id, &markers);

    pub_markers_.publish(markers);
    return true;
  }

  bool VisualServoingNode::liftPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    if (activated_) {
      activated_ = false;
    }
    Eigen::Vector3d waypoint_position = current_odometry_.position_W + Eigen::Vector3d(0.0, 0.0, 1.1);
    Eigen::Quaterniond waypoint_orientation = mav_msgs::quaternionFromYaw(start_yaw_);
    Eigen::Vector3d velocity_command (0.0, 0.0, 0.0);
    Eigen::Vector3d ang_velocity_command (0.0, 0.0, 0.0);
    double duration = 5e9;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    generateTrajectoryMsg(trajectory_msg, waypoint_position, 
                          waypoint_orientation, velocity_command, 
                          ang_velocity_command, duration);
    pub_trajectory_.publish(trajectory_msg);

    // get marker to display Waypoint in RVIZ
    visualization_msgs::MarkerArray markers;
    double marker_distance = 0.02; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mav_trajectory_generation::drawMavSampledTrajectory(states_, marker_distance, frame_id, &markers);

    pub_markers_.publish(markers);
    return true;
  }


  void VisualServoingNode::loadParams() {
    if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
      ROS_WARN("[VisualServoingNode] param max_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
      ROS_WARN("[VisualServoingNode] param max_a not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/max_ang_v", max_ang_v_)){
      ROS_WARN("[VisualServoingNode] param max_ang_v not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/max_ang_a", max_ang_a_)){
      ROS_WARN("[VisualServoingNode] param max_ang_a not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/sampling_time", sampling_time_)){
      ROS_WARN("[VisualServoingNode] param sampling_time not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/k_p", k_p_)){
      ROS_WARN("[VisualServoingNode] param k_p not found");
    }
    if (!nh_.getParam(ros::this_node::getName() + "/k_p_ang", k_p_ang_)){
      ROS_WARN("[VisualServoingNode] param k_p_ang not found");
    }
  }

  void VisualServoingNode::loadTFs() {
    ROS_INFO_ONCE("[VisualServoingNode] loading TFs");
    try
      {
        tf_listener_.waitForTransform("base", "imu", ros::Time(0),
                                      ros::Duration(5.0));
        tf_listener_.lookupTransform("base", "imu",  
                                    ros::Time(0), tf_base_imu_);
        tf::transformTFToEigen(tf_base_imu_, T_B_imu_);
        ROS_INFO_STREAM("[VisualServoingNode] Found base to imu transform!");
      }
    catch (tf::TransformException ex)
      {
        ROS_ERROR("[VisualServoingNode] %s",ex.what());
      }
    try
      {
        tf_listener_.waitForTransform("base", "cam", ros::Time(0),
                                      ros::Duration(5.0));
        tf_listener_.lookupTransform("base", "cam",  
                                    ros::Time(0), tf_base_cam_);
        Eigen::Affine3d T_B_cam;
        tf::transformTFToEigen(tf_base_cam_, T_B_cam);
        R_B_cam_ = T_B_cam.rotation();
        t_B_cam_ = T_B_cam.translation();
        ROS_INFO_STREAM("[VisualServoingNode] Found base to cam transform!");
      }
    catch (tf::TransformException ex)
      {
        ROS_ERROR("[VisualServoingNode] %s",ex.what());
      }
  }
  
  bool VisualServoingNode::activateServoingSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    start_yaw_ = mav_msgs::yawFromQuaternion(current_odometry_.orientation_W_B);
    start_position_ = current_odometry_.position_W;
    velocity_integral_ = Eigen::Vector3d::Zero();
    angular_velocity_integral_ = 0.0;
    t_last_run_ = ros::Time::now();

    if (activated_){
      ROS_INFO_STREAM("[VisualServoingNode] DE-ACTIVATED SERVOING");
      activated_ = false;
    }
    else {
      ROS_INFO_STREAM("[VisualServoingNode] ACTIVATED SERVOING");
      activated_ = true;
    }
    return true;
  }

  void VisualServoingNode::run(const ros::TimerEvent& event) {

    if (!received_odometry_  || !activated_)
      return;

    // get error vector 
    Eigen::Vector3d goal = current_pole_pos_ + Eigen::Vector3d(0.0, 0.0, 1.8);
    Eigen::Vector3d error = goal - current_odometry_.position_W;

    ros::Time t_now = ros::Time::now();
    double sampling_time = (t_now - t_last_run_).toSec();
    t_last_run_ = t_now;

    Eigen::Vector3d velocity_command = error * k_p_; // alternative: error.pow(1/3) * k_p_

    if(velocity_command.norm() > max_v_){
      ROS_INFO_STREAM("[VisualServoingNode] velocity_command > max_v_");
      velocity_command = velocity_command.normalized() * max_v_;
    }

    velocity_integral_ += velocity_command * sampling_time;


    Eigen::Vector3d waypoint_position = start_position_ + velocity_integral_;

    double yaw_velocity_command;

    double yaw_cam = - 2 / 3 * M_PI;
    double yaw_desired = std::atan2(error(1), error(0)) - yaw_cam;

    double current_yaw = mav_msgs::yawFromQuaternion(current_odometry_.orientation_W_B);
    double yaw_error = yaw_desired - current_yaw;

    if(yaw_error > M_PI){
      ROS_INFO_STREAM("[VisualServoingNode] yaw_error > M_PI");
      yaw_error = yaw_error - 2 * M_PI;
    }
    if(yaw_error < -M_PI){
      ROS_INFO_STREAM("[VisualServoingNode] yaw_error < -M_PI");
      yaw_error = yaw_error + 2* M_PI;
    }

    if(error.norm() < 0.1){
      ROS_INFO_STREAM("[VisualServoingNode] Error Norm < 0.1");
      yaw_velocity_command = 0.0;
    } 
    else {
      yaw_velocity_command = yaw_error * k_p_ang_;
    }

    angular_velocity_integral_ += yaw_velocity_command * sampling_time;

    double waypoint_yaw = start_yaw_ + angular_velocity_integral_;

    Eigen::Quaterniond waypoint_orientation = mav_msgs::quaternionFromYaw(waypoint_yaw);
    Eigen::Vector3d waypoint_ang_velocity = Eigen::Vector3d(0.0, 0.0, yaw_velocity_command);

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    double duration = 0.0;
    generateTrajectoryMsg(trajectory_msg, waypoint_position, 
                          waypoint_orientation, velocity_command, 
                          waypoint_ang_velocity, duration);

    pub_trajectory_.publish(trajectory_msg);

    // get marker to display Waypoint in RVIZ
    visualization_msgs::MarkerArray markers;
    double marker_distance = 0.02; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mav_trajectory_generation::drawMavSampledTrajectory(states_, marker_distance, frame_id, &markers);

    pub_markers_.publish(markers);
  }

  void VisualServoingNode::generateTrajectoryMsg(trajectory_msgs::MultiDOFJointTrajectory& msg,
                    const Eigen::Vector3d& waypoint_position,
                    const Eigen::Quaterniond& waypoint_orientation,
                    const Eigen::Vector3d& velocity_command,
                    const Eigen::Vector3d& ang_velocity_command,
                    double duration) {
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    Eigen::Vector3d acceleration_des(0.0, 0.0, 0.0);
    Eigen::Vector3d jerk_des(0.0, 0.0, 0.0);
    Eigen::Vector3d snap_des(0.0, 0.0, 0.0);
    Eigen::Vector3d angular_velocity_des(0.0, 0.0, 0.0);
    Eigen::Vector3d angular_accel_des(0.0, 0.0, 0.0);
    Eigen::Vector3d force_des(0.0, 0.0, 0.0);
    Eigen::Vector3d torque_des(0.0, 0.0, 0.0);

    mav_msgs::EigenTrajectoryPoint trajectory_point(
        duration, waypoint_position, velocity_command, acceleration_des, jerk_des, snap_des,
        waypoint_orientation, angular_velocity_des, angular_accel_des, force_des, torque_des);

    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);

    states_.clear();  
    states_.push_back(trajectory_point);
  }

} //namespace geranos

template<typename... Ts>
std::shared_ptr<geranos::VisualServoingNode> makeNode(Ts&&... params) {
  std::shared_ptr<geranos::VisualServoingNode> Node(new geranos::VisualServoingNode(std::forward<Ts>(params)...));
  return Node;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_servoing_node");

  ros::NodeHandle nh, private_nh("~");

  auto Node = makeNode(nh, private_nh);

  ros::spin();

  return 0;
}
