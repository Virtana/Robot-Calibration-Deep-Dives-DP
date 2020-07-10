#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_joint_state_pub");

  ros::NodeHandle nodeHandler;

  ros::Publisher joint_states_pub = nodeHandler.advertise<sensor_msgs::JointState>("joint_states", 20);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    sensor_msgs::JointState joint_states_message;

    joint_states_message.name = {"base_to_link_1","link_1_to_link_2"};
    joint_states_message.position = {0,0};
    joint_states_message.header.stamp = ros::Time::now();

    joint_states_pub.publish(joint_states_message);
 
    ros::spinOnce();
 
    loop_rate.sleep();

  }

  return 0;
}

