#ifndef ROBOT_CALIBRATION_3D_JOINT_STATE_PUBLISHER_H_
#define ROBOT_CALIBRATION_3D_JOINT_STATE_PUBLISHER_H_

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "time.h"
#include "cmath"

class JointStatePublisher
{
public:
  // constructor takes a NodeHandle as its sole argument
  JointStatePublisher(ros::NodeHandle nh);
  // used to perform actual publishing of message
  void publishJointStateMessage();

private:
  ros::NodeHandle nh_;

  // publisher to do actual publishing will be obtained via the NodeHandle in the constructor
  ros::Publisher joint_states_pub_;

  // TODO: Retrieve this number from the URDF in constructor, instead of hardcoding in constructor
  int num_joints_;

  // parameters to govern state updates, retrieved from param server in constructor
  int num_state_changes_;
  double state_update_interval_float_;

  // keep track of how many messages have been sent
  int count_;

  // time of last joint update
  ros::Time last_update_time_;

  // message to be modified at every state update and sent to /joint_states
  sensor_msgs::JointState joint_states_message_;

  // simple function to get random angle within supplied limits
  double randomAngle(double lower_limit, double upper_limit);

  // function to update joint state message before it is published
  void updateJointStateMessage();
};

#endif
