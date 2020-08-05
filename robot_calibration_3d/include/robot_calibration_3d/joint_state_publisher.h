#ifndef ROBOT_CALIBRATION_3D_JOINT_STATE_PUBLISHER_H_
#define ROBOT_CALIBRATION_3D_JOINT_STATE_PUBLISHER_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "time.h"
#include "cmath"

class JointStatePublisher
{
  // constructor takes a NodeHandle as its sole argument
public:
  JointStatePublisher(ros::NodeHandle nh);

private:
  ros::NodeHandle nh_;
  // publisher to do actual publishing will be obtained via the NodeHandle in the constructor
  ros::Publisher joint_states_pub_;
  // TODO: Retrieve this number from the URDF in constructor, instead of hardcoding.
  int num_joints_;
  // parameters to govern state updates, retrieved from param server in constructor
  int num_state_changes_;
  double state_update_interval_float_;
  // message to be modified at every state update and sent to /joint_states
  sensor_msgs::JointState joint_states_message_;
};

// simple function to get random angle within supplied limits
double randomAngle(double lower_limit, double upper_limit);
void updateJointStateMessage(sensor_msgs::JointState* msg);

#endif  
