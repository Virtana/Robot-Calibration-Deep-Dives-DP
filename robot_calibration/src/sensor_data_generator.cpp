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

  double link_1_length, link_2_length;
  link_1_length = 1.8;
  link_2_length = 1.0;

  double ee_position[] = {(link_1_length * cos(angles[0])) + (link_2_length * cos(angles[0] + angles[1])) , (link_1_length * sin(angles[0])) + (link_2_length * sin(angles[0] + angles[1]))}; 
  
  angles[0] += 0.044; //theta1 offset
  angles[1] += 0.013; //theta2 offset

  YAML::Emitter yaml_out_stream;
  std::ofstream outfile;

  yaml_out_stream << YAML::BeginMap;
  yaml_out_stream << YAML::Key  << "Angles";
  yaml_out_stream << YAML::Value;
  yaml_out_stream << YAML::Flow;
  yaml_out_stream << YAML::BeginSeq << angles[0] << angles[1] << YAML::EndSeq;
  yaml_out_stream << YAML::Key  << "End Effector Position";
  yaml_out_stream << YAML::Value;
  yaml_out_stream << YAML::Flow;
  yaml_out_stream << YAML::BeginSeq <<  ee_position[0] << ee_position[1] << YAML::EndSeq;
  yaml_out_stream << YAML::EndMap;

  outfile.open("/home/jad/catkin_ws/src/Robot-Calibration-Deep-Dives-DP/robot_calibration/sensor_data/sensor_data" + std::to_string(ros::Time::now().toSec()) + ".yaml", std::fstream::app);
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
