#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <fstream>

void calculateAndWriteData(const sensor_msgs::JointState::ConstPtr& msg)
{
  double angles[2];
  angles[0] = msg->position[0];
  angles[1] = msg->position[1];

  double ee_position[] = {(1.8 * cos(angles[0])) + (1 * cos(angles[0] + angles[1])) , (1.8 * sin(angles[0])) + (1 * sin(angles[0] + angles[1]))}; //lengths currently hardcoded
  
  angles[0] += 6.44; //theta1 offset
  angles[1] += 3.13; //theta2 offset

  YAML::Emitter yaml_out_stream;
  std::ofstream outfile;
  
  //yaml_out_stream << YAML::Flow;
  yaml_out_stream << YAML::BeginSeq << angles/*[0] << angles[1]*/ << YAML::EndSeq;
  yaml_out_stream << YAML::BeginSeq << ee_position/*[0] << ee_position[1] */<< YAML::EndSeq;

  outfile.open("sensor_data.yaml", std::fstream::app);
  outfile << yaml_out_stream.c_str();
  outfile.close();

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_joint_state_listener");

  ros::NodeHandle nh;

  ros::Subscriber angles_subscriber = nh.subscribe("joint_states", 100, calculateAndWriteData);

  ros::spin();

  return 0;
}
