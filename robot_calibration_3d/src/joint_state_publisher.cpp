#include "robot_calibration_3d/joint_state_publisher.h"

namespace joint_state_publisher
{
JointStatePublisher::JointStatePublisher(ros::NodeHandle nh) : nh_(nh)
{
  // currently hardcoding this, should be obtained from URDF
  num_joints_ = 6;

  // getting publisher
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 20);

  // retrieving state update parameters from param server
  nh_.param("robot_joint_angles/num_state_changes", num_state_changes_, 50);
  nh_.param("robot_joint_angles/state_update_interval", state_update_interval_float_, 3.0);

  // ensure the update interval is positive
  if (state_update_interval_float_ <= 0)
  {
    ROS_WARN("%s", "Non-positive state update interval specified, interval updated to default.");
    state_update_interval_float_ = 3.0;
  }

  // getting update interval as a ros::Duration rather than a double eases calculation
  ros::Duration state_update_interval_duration(state_update_interval_float_);

  // set rate at which messages are sent, regardless of when state is updated
  ros::Rate loop_rate_(10);

  // seed number generator to ensure randomness
  srand(time(0));

  // populate the joint name array of the message, assuming the URDF uses the naming convention that the ith joint is
  // named "joint_i"
  for (int i = 1; i <= num_joints_; i++)
    joint_states_message_.name.push_back("joint_" + std::to_string(i));

  // initialise the joint state array of the message with random values for first update
  // curently it is hardcoded to account for the unique limits of rotation of each joint
  joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-2.9671, 2.9671));
  joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-1.0472, 2.4435));
  joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-2.4784, 4.0143));
  joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-3.3161, 3.3161));
  joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-2.0944, 2.0944));
  joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-6.2832, 6.2832));
  // for (int i = 0; i < num_joints_; i++)
  //   joint_states_message_.position.push_back(((((rand() - double(RAND_MAX) / 2) / double(RAND_MAX / 2))) * 2 *
  //   M_PI));

  // set time of update after this first update
  ros::Time last_update_time = ros::Time::now();

  // start counting updates
  int count = 1;

  while (ros::ok())
  {
    // if enough time has passed since last update and number of desired state changes has not yet been met, update
    // state of robot
    if (ros::Time::now() - last_update_time >= state_update_interval_duration && count < num_state_changes_)
    {
      // clearing joint state array (actually a vector) to facilitate the pushing of 6 new angles
      joint_states_message_.position.erase(joint_states_message_.position.begin(),
                                           joint_states_message_.position.end());

      // fill array with 6 new random angles
      // curently it is hardcoded to account for the unique limits of rotation of each joint
      joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-2.9671, 2.9671));
      joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-1.0472, 2.4435));
      joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-2.4784, 4.0143));
      joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-3.3161, 3.3161));
      joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-2.0944, 2.0944));
      joint_states_message_.position.push_back(joint_state_publisher::randomAngle(-6.2832, 6.2832));
      // for (int i = 0; i < num_joints_; i++)
      //   joint_states_message_.position.push_back(
      //       ((((rand() - double(RAND_MAX) / 2) / double(RAND_MAX / 2))) * 2 * M_PI));

      // update the time of last update, as well as update count
      last_update_time += state_update_interval_duration;
      ++count;
    }

    // timestamp the message
    joint_states_message_.header.stamp = ros::Time::now();

    // actual publishing
    joint_states_pub_.publish(joint_states_message_);

    // wait a certan amount of time (determined by loop rate) before sending another message
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// simple function to get random angle within supplied limits
double randomAngle(double lower_limit, double upper_limit)
{
  return lower_limit + (double(rand()) / double(RAND_MAX)) * (upper_limit - lower_limit);
}

}  // namespace joint_state_publisher
