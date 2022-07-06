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
      odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &VisualServoingNode::odometryCallback, this);
      pole_vicon_sub_ = nh_.subscribe("geranos_pole_white/vrpn_client/estimated_transform", 1, &VisualServoingNode::poleViconCallback, this);
      pose_estimate_sub_ = nh_.subscribe("PolePoseNode/EstimatedPose", 1, &VisualServoingNode::poseEstimateCallback, this);
      // create publisher for trajectory
      pub_trajectory_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 0);
      // create publisher for estimated pole position
      pole_pos_pub_ = nh_.advertise<geometry_msgs::PointStamped>(ros::this_node::getName() + "/estimated_pole_position", 0);
      // create publisher for RVIZ markers
      pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/trajectory_markers", 0);
      // create publisher for estimation error
      error_pub_ = nh_.advertise<geometry_msgs::PointStamped>(ros::this_node::getName() + "/error_vector", 0);

      timer_ = nh_.createTimer(ros::Duration(0.5), &VisualServoingNode::run, this);

      activate_service_ = nh_.advertiseService("activate_servoing_service", &VisualServoingNode::activateServoingSrv, this);

      loadParams();
      loadTFs();

      traj_state_ = TrajectoryState::TRAJ_3D;
    }

  VisualServoingNode::~VisualServoingNode() {}

  void VisualServoingNode::odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("VisualServoingNode received first odometry!");
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
    transformOdometry(current_odometry_);
    received_odometry_ = true;
  }

  void VisualServoingNode::transformOdometry(mav_msgs::EigenOdometry& odometry) {
    Eigen::Matrix3d R_B_imu = T_B_imu_.rotation();  // rotation from imu to body frame
    Eigen::Vector3d r_B_imu_imu = T_B_imu_.translation();  // body to imu offset expressed in base frame
    Eigen::Matrix3d R_W_B = odometry.orientation_W_B.toRotationMatrix();
    // add translational offset between imu and body frame
    odometry.position_W -= R_W_B * R_B_imu * r_B_imu_imu;
  }

  void VisualServoingNode::poseEstimateCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // transform pose to base frame
    Eigen::Vector3d pole_pos_C = mav_msgs::vector3FromPointMsg(pose_msg->pose.position);
    Eigen::Vector3d pole_pos_B = t_B_cam_ + R_B_cam_ * pole_pos_C;

    // transform position to world frame
    Eigen::Matrix3d R_W_B = current_odometry_.orientation_W_B.toRotationMatrix();
    current_pole_pos_ = current_odometry_.position_W + R_W_B * pole_pos_B;
    received_pole_pose_ = true;
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
    if (activated_)
      activated_ = false;
    else 
      activated_ = true;
    return true;
  }

  bool VisualServoingNode::calc3DTrajectory(mav_trajectory_generation::Trajectory* trajectory){
    Eigen::Vector3d start_pos, start_vel;
    start_pos = current_odometry_.position_W;
    start_vel = current_odometry_.velocity_B;

    Eigen::Vector3d goal_pos, goal_vel;
    goal_pos << current_pole_pos_vicon_(0), current_pole_pos_vicon_(1), current_pole_pos_vicon_(2) + 1.8;
    goal_vel = Eigen::Vector3d::Zero();

    return planTrajectory(goal_pos, goal_vel, start_pos, start_vel, max_v_, max_a_, trajectory);
  }

  bool VisualServoingNode::calc4DTrajectory(mav_trajectory_generation::Trajectory* trajectory){
    double current_yaw = mav_msgs::yawFromQuaternion(current_odometry_.orientation_W_B);

    Eigen::Vector4d start_pos, start_vel;
    start_pos << current_odometry_.position_W, current_yaw;
    start_vel << current_odometry_.velocity_B, current_odometry_.angular_velocity_B(2);

    Eigen::Vector4d goal_vel, goal_pos;
    goal_vel << 0.0, 0.0, 0.0, 0.0;
    goal_pos << current_pole_pos_vicon_(0), current_pole_pos_vicon_(1), current_pole_pos_vicon_(2) + 1.8, current_yaw;

    return planTrajectory(goal_pos, goal_vel, start_pos, start_vel, max_v_, max_a_, trajectory);

  }
  bool VisualServoingNode::calc6DTrajectory(mav_trajectory_generation::Trajectory* trajectory){
    return true;
  }


  void VisualServoingNode::run(const ros::TimerEvent& event) {
    ROS_INFO_STREAM("[VisualServoingNode] RUNNING");

    if (!received_odometry_ || !received_pole_pose_ || !activated_)
      return;

    mav_trajectory_generation::Trajectory trajectory;

    bool success = false;

    switch(traj_state_) {
      case TrajectoryState::TRAJ_3D:
        success = calc3DTrajectory(&trajectory);
        break;
      case TrajectoryState::TRAJ_4D:
        success = calc4DTrajectory(&trajectory);
        break;
      case TrajectoryState::TRAJ_6D:
        success = calc6DTrajectory(&trajectory);
        break;
      default:
        ROS_ERROR_STREAM("traj_state_ is set to wrong state");
    }

    if (!success) {
      ROS_ERROR_STREAM("[VisualServoingNode] Failed to plan Trajectory!");
      return;
    }

    // get markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";
    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    
    // Sample:
    states_.clear();
    mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_time_, &states_);

    // set yaw manually for 3D Trajectory
    if (traj_state_ == TrajectoryState::TRAJ_3D) {
      double current_yaw = mav_msgs::yawFromQuaternion(current_odometry_.orientation_W_B);
      for (auto state : states_) {
       ROS_INFO_STREAM("Yaw before = " << state.getYaw());
       state.setFromYaw(current_yaw);
       ROS_INFO_STREAM("Yaw after = " << state.getYaw());
      }
    }

    publishTrajectory(trajectory, markers);
  }

  // Plans a trajectory from a start position and velocity to a goal position and velocity
  bool VisualServoingNode::planTrajectory(const Eigen::VectorXd& goal_pos,
                                      const Eigen::VectorXd& goal_vel,
                                      const Eigen::VectorXd& start_pos,
                                      const Eigen::VectorXd& start_vel,
                                      double v_max, double a_max,
                                      mav_trajectory_generation::Trajectory* trajectory) {
    ROS_INFO_STREAM("[VisualServoingNode] PLAN TRAJECTORY");
    assert(trajectory);
    const int dimension = goal_pos.size();
    // Array for all waypoints and their constraints
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
        mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // start = desired start vector
    // end = desired end vector
    mav_trajectory_generation::Vertex start(dimension), end(dimension);

    /******* Configure start point *******/
    start.makeStartOrEnd(start_pos, derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel);
    vertices.push_back(start);

    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to zero
    end.makeStartOrEnd(goal_pos, derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));
    trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
    
    return true;
  }

  void VisualServoingNode::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory, const visualization_msgs::MarkerArray& markers) {
    pub_markers_.publish(markers);
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(states_, &trajectory_msg);
    trajectory_msg.header.frame_id = "world";
    trajectory_msg.header.stamp = ros::Time::now();
    pub_trajectory_.publish(trajectory_msg);
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
