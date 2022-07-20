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
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <omav_local_planner/ExecuteTrajectory.h>


// tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// tf2
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// srvs
#include <std_srvs/Empty.h>

#define M_PI 3.14159265358979323846

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
		void initializeSubscribers();
		void initializePublishers();
		void initializeServices();
		void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
		void poseEstimateCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
		void poleViconCallback(const geometry_msgs::TransformStamped& pole_transform_msg);

		bool activateServoingSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	  	bool grabPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  		bool liftPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  		void generateTrajectoryMsg(trajectory_msgs::MultiDOFJointTrajectory& msg,
  									const Eigen::Vector3d& waypoint_position,
  									const Eigen::Quaterniond& waypoint_orientation,
  									const Eigen::Vector3d& velocity_command,
  									const Eigen::Vector3d& ang_velocity_command,
  									double duration);

		void loadParams();
		void loadTFs();

		void transformPose();
		void transformOdometry(mav_msgs::EigenOdometry& odometry);
		void publishOdometry(const mav_msgs::EigenOdometry& odom);



		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber odometry_sub_;
		ros::Subscriber pose_estimate_sub_;
		ros::Subscriber pole_vicon_sub_;
		ros::Publisher pub_trajectory_;
		ros::Publisher pub_markers_;
		ros::Publisher pole_pos_pub_;
		ros::Publisher error_pub_;
		ros::Publisher transformed_odom_pub_;

		ros::Timer timer_run_;
		ros::Timer timer_update_;

		ros::ServiceServer activate_service_;
		ros::ServiceServer grab_pole_service_;
		ros::ServiceServer lift_pole_service_;

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
		Eigen::Vector3d current_pole_pos_B_;
		mav_msgs::EigenTrajectoryPoint pole_trajectory_point_;
		Eigen::Vector3d current_pole_pos_vicon_;

		Eigen::Vector3d start_position_;
		Eigen::Vector3d velocity_integral_;

		Eigen::Vector3d waypoint_position_;
		Eigen::Quaterniond waypoint_orientation_;

		bool received_odometry_;
		bool received_pole_pose_;
		bool activated_;

		double max_v_; // m/s
		double max_a_; // m/s^2
		double max_ang_v_;
		double max_ang_a_;
		double sampling_time_;

		double start_yaw_;
		double current_yaw_;
		
		double angular_velocity_command_;
		double angular_velocity_integral_;

		double k_p_;
		double k_p_ang_;

		ros::Time t_last_run_;

		mav_msgs::EigenTrajectoryPoint::Vector states_;
		TrajectoryState traj_state_;

	};
}