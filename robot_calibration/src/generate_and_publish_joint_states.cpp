#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_joint_state_pub");

  ros::NodeHandle node;

  ros::Publisher joint_states_pub = node.advertise<sensor_msgs::JointState>("joint_states", 20);

  int num_messages;
  node.param("num_messages", num_messages, 50);
 
  double message_interval;
  node.param("message_interval", message_interval, 0.1);
  
  ros::Rate loop_rate(1/message_interval);

  int count = 0;
  srand(time(0));
  while(ros::ok() && count < num_messages)
  {
    sensor_msgs::JointState joint_states_message;

    joint_states_message.name = {"base_to_link_1","link_1_to_link_2"};

    double theta1 = (double(rand()) / (double(RAND_MAX))) * 6.28;
    double theta2 = (double(rand()) / (double(RAND_MAX))) * 6.28;

    joint_states_message.position = {theta1, theta2};
    joint_states_message.header.stamp = ros::Time::now();

    joint_states_pub.publish(joint_states_message);
 
    ros::spinOnce();
    loop_rate.sleep();
    ++count; 
  } 
  return 0;
}

