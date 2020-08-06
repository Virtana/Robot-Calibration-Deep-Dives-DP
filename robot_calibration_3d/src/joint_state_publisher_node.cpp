#include "robot_calibration_3d/joint_state_publisher.h"

int main(int argc, char** argv)
{
  // initialise ros
  ros::init(argc, argv, "joint_state_publisher_node");

  // NodeHandle to be passed to JointStatePublisher
  ros::NodeHandle nh;

  JointStatePublisher joint_state_publisher(nh);

  // rate at which messages are sent, regardless of when state is updated, set to 10 Hertz
  ros::Rate loop_rate(10);

  // start counting updates
  int count = 0;

  while (ros::ok())
  {
    count = joint_state_publisher.publishJointStateMessage(count);
  }

  // wait a certan amount of time (determined by loop rate) before sending another message
  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}
