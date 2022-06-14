#include <geranos_visual_servoing/visual_servoing_node.h>

namespace geranos {
  VisualServoingNode::VisualServoingNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) : 
    nh_(nh),
    private_nh_(private_nh),
    tf2_(buffer_) {
      odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &VisualServoingNode::odometryCallback, this);
      pose_estimate_sub_ = nh_.subscribe("PolePoseNode/EstimatedPose", 1, &VisualServoingNode::poseEstimateCallback, this);
      pub_trajectory_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(ros::this_node::getName() + "/trajectory", 0);
      loadParams();
      loadTFs();
    }

  VisualServoingNode::~VisualServoingNode() {}

  void VisualServoingNode::odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("VisualServoingNode received first odometry!");
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
    transformOdometry(current_odometry_);
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
    geometry_msgs::PoseStamped pose_msg_B;
    try 
    {
      buffer_.transform(*pose_msg, pose_msg_B, "base");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("[VisualServoingNode] %s", ex.what());
    }
    // transform position to world frame
    Eigen::Vector3d pole_pos_B = mav_msgs::vector3FromPointMsg(pose_msg_B.pose.position);
    // Eigen::Matrix3d pole_rot_B = mav_msgs::quaternionFromMsg(pose_msg_B.pose.orientation).toRotationMatrix();
    Eigen::Matrix3d R_W_B = current_odometry_.orientation_W_B.toRotationMatrix();
    current_pole_pos_ = current_odometry_.position_W + R_W_B * pole_pos_B;
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
        tf::transformTFToEigen(tf_base_cam_, T_B_cam_);
        ROS_INFO_STREAM("[VisualServoingNode] Found base to cam transform!");
      }
    catch (tf::TransformException ex)
      {
        ROS_ERROR("[VisualServoingNode] %s",ex.what());
      }
  }

  void VisualServoingNode::run() {
    mav_trajectory_generation::Trajectory trajectory;
    Eigen::VectorXd goal_vel;
    goal_vel << 0.0, 0.0, 0.0;
    if(!planTrajectory(current_pole_pos_, goal_vel, 
                    current_odometry_.position_W, 
                    current_odometry_.velocity_B,
                    max_v_, max_a_, &trajectory)) {
      ROS_ERROR_STREAM("[VisualServoingNode] Failed to plan Trajectory!");
      return;
    }
    // Sample:
    states_.clear();
    mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_time_,
                                                   &states_);
    publishTrajectory(trajectory);
  }

  // Plans a trajectory from a start position and velocity to a goal position and velocity
  bool VisualServoingNode::planTrajectory(const Eigen::VectorXd& goal_pos,
                                      const Eigen::VectorXd& goal_vel,
                                      const Eigen::VectorXd& start_pos,
                                      const Eigen::VectorXd& start_vel,
                                      double v_max, double a_max,
                                      mav_trajectory_generation::Trajectory* trajectory) {
    assert(trajectory);
    const int dimension = goal_pos.size();
    assert(dimension == 3);
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

  void VisualServoingNode::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory) {
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
  ros::Rate rate(2); // Node Rate 2Hz

  auto Node = makeNode(nh, private_nh);

  while (ros::ok()) {
    Node->run();
    rate.sleep();
  }

  return 0;
}