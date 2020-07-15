#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>
#include <time.h>
#include <cmath>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_joint_state_pub");

  ros::NodeHandle nh;

  ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 20);

  int num_state_changes;
  nh.param("robot_joint_angles/num_state_changes", num_state_changes, 50);
 
  int state_update_interval;
  nh.param("robot_joint_angles/state_update_interval", state_update_interval, 3);
  
  ros::Rate loop_rate(10);

  int count = 0;

  srand(time(0));
  
  double theta1 = ((((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI);
  double theta2 = ((((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI);

  time_t last_update_time = time(0);
  
  while(ros::ok())
  {
    sensor_msgs::JointState joint_states_message;

    joint_states_message.name = {"base_to_link_1","link_1_to_link_2"};

    if(time(0) - last_update_time == state_update_interval && count < num_state_changes)
    {
    theta1 = (((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI;
    theta2 = (((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI; 
    last_update_time+=state_update_interval;
    ++count;
    }

    joint_states_message.position = {theta1, theta2};
    joint_states_message.header.stamp = ros::Time::now();

    joint_states_pub.publish(joint_states_message);
 
    ros::spinOnce();
    loop_rate.sleep();
  } 
  return 0;
}

