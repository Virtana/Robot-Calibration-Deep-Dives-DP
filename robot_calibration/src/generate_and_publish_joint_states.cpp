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
 
  double state_update_interval_float;
  nh.param("robot_joint_angles/state_update_interval", state_update_interval_float, 3.0);

  if(state_update_interval_float <= 0)
  {
    ROS_WARN("%s","Non-positive state update interval specified, interval updated to default.");
    state_update_interval_float = 3.0;
  }

  ros::Duration state_update_interval_duration(state_update_interval_float);
  
  ros::Rate loop_rate(10);

  int count = 0;

  srand(time(0));
  
  // double theta1 = ((((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI);
  // double theta2 = ((((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI);
  double theta1 = -0.5;
  double theta2 = -0.5;
  ros::Time last_update_time = ros::Time::now(); 
  
  while(ros::ok())
  {
    sensor_msgs::JointState joint_states_message;

    joint_states_message.name = {"base_to_link_1","link_1_to_link_2"};

    if(ros::Time::now() - last_update_time >= state_update_interval_duration && count < num_state_changes)
    {
      theta1 = (((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI;
      theta2 = (((rand() - double(RAND_MAX)/2) / double(RAND_MAX/2))) * 2*M_PI;
      
      last_update_time += state_update_interval_duration;
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

