#include <ros/ros.h>

// msgs
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/common.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mav_planning_msgs/PolynomialTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// Trajectories
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
// #include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

// tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// tf2
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// srvs
#include <std_srvs/Empty.h>

namespace geranos {
	enum TrajectoryState 
	{
		TRAJ_3D,
		TRAJ_4D,
		TRAJ_6D
	};

	class VisualServoingNode {
	public:
		VisualServoingNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~VisualServoingNode();

		void run(const ros::TimerEvent& event);
	private:
		void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
		void poseEstimateCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
		void poleViconCallback(const geometry_msgs::TransformStamped& pole_transform_msg);

		bool activateServoingSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		void loadParams();
		void loadTFs();

		void transformPose();
		void transformOdometry(mav_msgs::EigenOdometry& odometry);

		bool planTrajectory(const Eigen::VectorXd& goal_pos,
	                          const Eigen::VectorXd& goal_vel,
	                          const Eigen::VectorXd& start_pos,
	                          const Eigen::VectorXd& start_vel,
	                          double v_max, double a_max,
	                          mav_trajectory_generation::Trajectory* trajectory);
		bool calc3DTrajectory(mav_trajectory_generation::Trajectory* trajectory);
		bool calc4DTrajectory(mav_trajectory_generation::Trajectory* trajectory);
		bool calc6DTrajectory(mav_trajectory_generation::Trajectory* trajectory);
		void publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory, const visualization_msgs::MarkerArray& markers);

		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber odometry_sub_;
		ros::Subscriber pose_estimate_sub_;
		ros::Subscriber pole_vicon_sub_;
		ros::Publisher pub_trajectory_;
		ros::Publisher pub_markers_;
		ros::Publisher pole_pos_pub_;
		ros::Publisher error_pub_;

		ros::Timer timer_;

		ros::ServiceServer activate_service_;

		tf::TransformListener tf_listener_;
	  	tf::StampedTransform tf_base_imu_;
	  	tf::StampedTransform tf_base_cam_;
	  	Eigen::Affine3d T_B_imu_;
	  	Eigen::Matrix3d R_B_cam_;
	  	Eigen::Vector3d t_B_cam_;

	  	tf2_ros::Buffer buffer_;
  		tf2_ros::TransformListener tf2_;

		mav_msgs::EigenOdometry current_odometry_;
		Eigen::Vector3d current_pole_pos_;
		mav_msgs::EigenTrajectoryPoint pole_trajectory_point_;
		Eigen::Vector3d current_pole_pos_vicon_;

		bool received_odometry_;
		bool received_pole_pose_;
		bool activated_;

		double max_v_; // m/s
		double max_a_; // m/s^2
		double max_ang_v_;
		double max_ang_a_;
		double sampling_time_;

		double current_yaw_;

		mav_msgs::EigenTrajectoryPoint::Vector states_;
		TrajectoryState traj_state_;

	};
}