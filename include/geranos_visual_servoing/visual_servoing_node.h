#include <ros/ros.h>

//msgs
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/common.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_planning_msgs/PolynomialTrajectory.h>

// Trajectories
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

//tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace geranos {
	class VisualServoingNode {
	public:
		VisualServoingNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~VisualServoingNode();
	private:
		void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
		void poseEstimateCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

		void loadParams();
		void loadTFs();

		void transformPose();
		void transformOdometry();

		bool planTrajectory(const Eigen::VectorXd& goal_pos,
	                          const Eigen::VectorXd& goal_vel,
	                          const Eigen::VectorXd& start_pos,
	                          const Eigen::VectorXd& start_vel,
	                          double v_max, double a_max,
	                          mav_trajectory_generation::Trajectory* trajectory);
		void publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber odometry_sub_;
		ros::Subscriber pose_estimate_sub_;
		ros::Publisher pub_trajectory_;

		tf::TransformListener tf_listener_;
	  	tf::StampedTransform tf_base_imu_;
	  	tf::StampedTransform tf_base_cam_;
	  	Eigen::Affine3d T_B_imu_;
	  	Eigen::Affine3d T_B_cam_;

		mav_msgs::EigenOdometry current_odometry_;
		Eigen::Vector3d current_pole_pos_;
		Eigen::Quaterniond current_pole_rot_;

		double max_v_; // m/s
		double max_a_; // m/s^2
		double max_ang_v_;
		double max_ang_a_;

	};
}