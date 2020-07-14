#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


void writeAngleData(const sensor_msgs::JointState:ConstPtr& msg)
{
  double angles[2] = msg->position;
  
  for(double angle : angles)
    angle += 0.44;

  //need to write to file

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_joint_state_listener");

  ros::NodeHandle node;

  ros::Subscriber angles_subscriber = node.subscribe("joint_states", 100, writeAngleData);

  ros::spin();

  return 0;
}
