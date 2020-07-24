#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/JointState.h"
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "boost/filesystem.hpp"

class Listener
{
  public:
    void calculateAndWriteData(const sensor_msgs::JointState::ConstPtr& msg);
    std::string filename;
    double theta_1, theta_2, theta_1_offset, theta_2_offset;
    double ee_position[2];
    Listener(double t_1_offset, double t_2_offset)
    {
      filename = ros::package::getPath("robot_calibration");
      if(filename != "")
      {
        filename += "/sensor_data";
        boost::filesystem::create_directory(filename);
        filename += "/sensor_data_" + std::to_string((int)ros::Time::now().toSec()) + ".yaml";
      }
      theta_1 = -1.0;
      theta_2 = -1.0;

      ee_position[0] = 1000000000;
      ee_position[1] = 1000000000;

      theta_1_offset = t_1_offset;
      theta_2_offset = t_2_offset;
    }
};


void Listener::calculateAndWriteData(const sensor_msgs::JointState::ConstPtr& msg)
{
  double link_1_length, link_2_length;
  link_1_length = 1.8;
  link_2_length = 1.0;

  double angles[2];
  angles[0] = msg->position[0];
  angles[1] = msg->position[1];

  double ee_position[] = {(link_1_length * cos(angles[0])) + (link_2_length * cos(angles[0] + angles[1])) , (link_1_length * sin(angles[0])) + (link_2_length * sin(angles[0] + angles[1]))}; 
  
  angles[0] += this->theta_1_offset; 
  angles[1] += this->theta_2_offset;  

  if(angles[0] != this->theta_1 || angles[1] != this->theta_2)
  {
    this->theta_1 = angles[0];
    this->theta_2 = angles[1];

    YAML::Emitter yaml_out_stream;
    std::ofstream outfile;

    yaml_out_stream << YAML::Literal <<"\n";
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

    if(filename == "")
    {
      ROS_WARN("%s","Could not locate package in attempting to save sensor data.");
    }
    else
    {
      outfile.open(filename, std::fstream::app);
      outfile << yaml_out_stream.c_str();
      outfile.close();
    }   
  }
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_joint_state_listener"); 

  ros::NodeHandle nh;

  double theta_1_offset, theta_2_offset;

  nh.getParam("/theta_1_offset", theta_1_offset);
  nh.getParam("/theta_2_offset", theta_2_offset);

  Listener listener(theta_1_offset, theta_2_offset);

  ros::Subscriber angles_subscriber = nh.subscribe("joint_states", 10, &Listener::calculateAndWriteData, &listener);

  ros::spin();

  return 0;
}
