#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <cmath>
#include "yaml-cpp/yaml.h"


void writeAngleData(const sensor_msgs::JointState:ConstPtr& msg)
{
  double angles[] = msg->position;

  double ee_position[] = {(1.8 * cos(angles[0])) + (1 * cos(angles[0] + angles[1])) , (1.8 * sin(angles[0])) + (1 * sin(angles[0] + angles[1]))}; //lengths currently hardcoded
  
  angles[0] += 6.44; //theta1 offset
  angles[1] += 3.1; //theta2 offset

  YAML::Emitter yaml_out_stream;
  
  yaml_out_stream.open("sensor_readings.yaml");

  yaml_out_stream << YAML::Flow;
  yaml_out_stream << YAML::BeginSeq << angles/*[0] << angles[1]*/ << YAML::EndSeq;
  yaml_out_stream << YAML::BeginSeq << ee_position/*[0] << ee_position[1] */<< YAML::EndSeq;
  
  yaml_out_stream.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_joint_state_listener");

  ros::NodeHandle node;

  ros::Subscriber angles_subscriber = node.subscribe("joint_states", 100, writeAngleData);

  ros::spin();

  return 0;
}
