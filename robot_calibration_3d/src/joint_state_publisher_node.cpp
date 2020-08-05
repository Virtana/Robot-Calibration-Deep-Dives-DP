#include "robot_calibration_3d/joint_state_publisher.h"

int main(int argc, char** argv)
{
  // initialise ros
  ros::init(argc, argv, "joint_state_publisher_node");

  // NodeHandle to be passed to JointStatePublisher
  ros::NodeHandle nh;

  JointStatePublisher joint_state_publisher(nh);

  return 0;
}
