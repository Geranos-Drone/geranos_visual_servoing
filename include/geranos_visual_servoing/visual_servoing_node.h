#include <ros/ros.h>

//msgs
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/common.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace geranos {
	class VisualServoingNode {
	public:
		VisualServoingNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~VisualServoingNode();
	private:
		void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
		void poseEstimateCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

		void transformPose();
		void transformOdometry();

		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber odometry_sub_;
		ros::Subscriber pose_estimate_sub_;

		mav_msgs::EigenOdometry current_odometry_;
		Eigen::Vector3d current_pole_pos_;
		Eigen::Quaterniond current_pole_rot_;

	};
}