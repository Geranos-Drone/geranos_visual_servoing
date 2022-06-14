#include <geranos_visual_servoing/visual_servoing_node.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pole_trajectory_node");

  ros::NodeHandle nh, private_nh("~");

  ros::spin();

  return 0;
}