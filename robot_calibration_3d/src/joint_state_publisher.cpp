#include "robot_calibration_3d/joint_state_publisher.h"

JointStatePublisher::JointStatePublisher(ros::NodeHandle nh) : nh_(nh)
{
  // TODO: retrieve this value from the URDF rather than hardcode
  num_joints_ = 6;

  // initialising publisher
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 20);

  // retrieving state update parameters from param server
  nh_.param("robot_joint_angles/num_state_changes", num_state_changes_, 50);
  nh_.param("robot_joint_angles/state_update_interval", state_update_interval_float_, 3.0);

  // ensure the update interval is positive
  if (state_update_interval_float_ <= 0)
  {
    ROS_WARN("Non-positive state update interval specified (%f), interval updated to default (%f).",
             state_update_interval_float_, 3.0);
    state_update_interval_float_ = 3.0;
  }

  // seed number generator to ensure randomness
  srand(time(0));

  // populate the joint name array of the message, assuming the URDF uses the naming convention that the ith joint is
  // named "joint_i"
  // TODO: retrieve joint names directly from URDF, rather than assume the naming convention used below
  for (int i = 1; i <= num_joints_; i++)
    joint_states_message_.name.push_back("joint_" + std::to_string(i));

  // initialise the joint state array of the message with random values for first update
  // TODO: retrieve angle limits from URDF, rather than hardcoding limits as is done in this function
  updateJointStateMessage();

  // set time of last update, after this first update
  last_update_time_ = ros::Time::now();
}

// simple function to get random angle within supplied limits
double JointStatePublisher::randomAngle(double lower_limit, double upper_limit)
{
  return lower_limit + (double(rand()) / double(RAND_MAX)) * (upper_limit - lower_limit);
}

// function to update all joint state positions
// TODO: retrieve limits of rotation from URDF, rather than hardcoding
void JointStatePublisher::updateJointStateMessage()
{
  // clearing joint state array (actually a vector) to facilitate the pushing of 6 new angles
  joint_states_message_.position.erase(joint_states_message_.position.begin(), joint_states_message_.position.end());

  joint_states_message_.position.push_back(randomAngle(-2.9671, 2.9671));
  joint_states_message_.position.push_back(randomAngle(-1.0472, 2.4435));
  joint_states_message_.position.push_back(randomAngle(-2.4784, 4.0143));
  joint_states_message_.position.push_back(randomAngle(-3.3161, 3.3161));
  joint_states_message_.position.push_back(randomAngle(-2.0944, 2.0944));
  joint_states_message_.position.push_back(randomAngle(-6.2832, 6.2832));

  // timestamp the message
  joint_states_message_.header.stamp = ros::Time::now();

  return;
}

int JointStatePublisher::publishJointStateMessage(int count)
{
  // getting update interval as a ros::Duration rather than a double eases calculation
  ros::Duration state_update_interval_duration(state_update_interval_float_);

  // if enough time has passed since last update and number of desired state changes has not yet been met, update
  // state of robot
  if (ros::Time::now() - last_update_time_ >= state_update_interval_duration && count < num_state_changes_)
  {
    // fill array with 6 new random angles
    // TODO: retrieve angle limits from URDF, rather than hardcoding limits as is done in this function
    updateJointStateMessage();

    // update the time of last update, as well as update count
    last_update_time_ += state_update_interval_duration;

    // update count to return
    count++;
  }
  // actual publishing
  joint_states_pub_.publish(joint_states_message_);
  return count;
}
