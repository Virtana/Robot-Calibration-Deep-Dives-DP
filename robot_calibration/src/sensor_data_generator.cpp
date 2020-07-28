#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/JointState.h"
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "boost/filesystem.hpp"

//class to facilitate calculation and writing of data
class Listener
{
  std::string filename;
  double theta_1, theta_2, theta_1_offset, theta_2_offset, link_1_length, link_2_length, ee_position[2];
  public:
  void calculateAndWriteData(const sensor_msgs::JointState::ConstPtr& msg);
  Listener(double theta_1_offset, double theta_2_offset)
  {
    filename = ros::package::getPath("robot_calibration");

    //if package could not be found, filename will be empty and the node will shutdown
    if(filename != "")
    {
      filename += "/sensor_data";
      boost::filesystem::create_directory(filename);
      filename += "/sensor_data_" + std::to_string((int)ros::Time::now().toSec()) + ".yaml";
    }
    else
    {
      ros::shutdown();
    }
    
    //link lengths hardcoded
    link_1_length = 1.8;
    link_2_length = 1.0;

    //initial values of theta_1 and theta_2 before a joint state message has been received
    theta_1 = 7;
    theta_2 = 7;

    //initial ee_position calculated based on above joint angle values
    ee_position[0] = (link_1_length * cos(theta_1)) + (link_2_length * cos(theta_1 + theta_2));
    ee_position[1] = (link_1_length * sin(theta_1)) + (link_2_length * sin(theta_1 + theta_2));

    //setting the angle offsets which are supplied to class as parameters on creation
    this->theta_1_offset = theta_1_offset;
    this->theta_2_offset = theta_2_offset;
  }
};


void Listener::calculateAndWriteData(const sensor_msgs::JointState::ConstPtr& msg)
{
  //receiving values of theta_1 and theta_2 sent from the generator/publisher node via /joint_states topic
  double theta_1_from_message = msg->position[0]; 
  double theta_2_from_message = msg->position[1];

  //check if joint state has actually been changed (if class member variables differ from the angles sent via message (class member variable would have offset applied))
  if(this->theta_1 != theta_1_from_message + this->theta_1_offset || this->theta_2 != theta_2_from_message + this->theta_2_offset)
  {

    //calculate new end effector position with new angles
    double ee_position[] = {(this->link_1_length * cos(theta_1_from_message)) + (this->link_2_length * cos(theta_1_from_message + theta_2_from_message )) , (this->link_1_length * sin(theta_1_from_message)) + (this->link_2_length * sin(theta_1_from_message + theta_2_from_message ))};  
    
    //update class variables before writing to file
    this->theta_1 = theta_1_from_message + this->theta_1_offset;
    this->theta_2 = theta_2_from_message + this->theta_2_offset;
    
    //write the offset angles and the true end effector position to file
    YAML::Emitter yaml_out_stream;
    std::ofstream outfile;

    yaml_out_stream << YAML::Literal <<"\n";
    yaml_out_stream << YAML::BeginMap;
    yaml_out_stream << YAML::Key  << "Angles";
    yaml_out_stream << YAML::Value;
    yaml_out_stream << YAML::Flow;
    yaml_out_stream << YAML::BeginSeq << this->theta_1 << this->theta_2 << YAML::EndSeq;
    yaml_out_stream << YAML::Key  << "End Effector Position";
    yaml_out_stream << YAML::Value;
    yaml_out_stream << YAML::Flow;
    yaml_out_stream << YAML::BeginSeq <<  ee_position[0] << ee_position[1] << YAML::EndSeq;
    yaml_out_stream << YAML::EndMap;


    outfile.open(filename, std::fstream::app);
    outfile << yaml_out_stream.c_str();
    outfile.close();
      
  }
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_joint_state_listener"); 

  ros::NodeHandle nh;

  double theta_1_offset, theta_2_offset;
  
  //retrieve offsets from parameter server, which are set via launch file
  nh.getParam("/theta_1_offset", theta_1_offset);
  nh.getParam("/theta_2_offset", theta_2_offset);

  //initialise listener class with offsets as parameters
  Listener listener(theta_1_offset, theta_2_offset);

  //make subscriber
  ros::Subscriber angles_subscriber = nh.subscribe("joint_states", 10, &Listener::calculateAndWriteData, &listener);

  ros::spin();

  return 0;
}
