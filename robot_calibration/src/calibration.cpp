#include "ceres/ceres.h"
#include "yaml-cpp/yaml.h"
#include "boost/filesystem.hpp"
#include "ros/ros.h"
#include <string>


// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::Problem;
// using ceres::Solver;
// using ceres::Solve;

// double *endEffectorPosition(double theta_1, double theta_2, double link_1_length, double link_2_length)
// {
//     double ee[2] = {(link_1_length * cos(theta_1)) + (link_2_length * cos(theta_1 + theta_2)) , (link_1_length * sin(theta_1)) + (link_2_length * sin(theta_1 + theta_2))};

//     return ee;
// }


class CostFunctor
{
  std::string filename;
  public:
  CostFunctor(std::string filename)
  {
    YAML::Node yaml_node = YAML::LoadFile("/home/jad/catkin_ws/src/Robot-Calibration-Deep-Dives-DP/robot_calibration/sensor_data/fake_testing.yaml");

    if(yaml_node.Type() == YAML::NodeType::Null)
      ROS_WARN("%s","null");
    if(yaml_node.Type() == YAML::NodeType::Scalar)
      ROS_WARN("%s","scalar");
    if(yaml_node.Type() == YAML::NodeType::Sequence)
      ROS_WARN("%s","sequence");
    if(yaml_node.Type() == YAML::NodeType::Map)
      ROS_WARN("%s","map");
    if(yaml_node.Type() == YAML::NodeType::Undefined)
      ROS_WARN("%s","undefined");
      

    size_t i = 0;
    ROS_WARN("%F", yaml_node[i]["Angles"][i].as<float>());
    size_t j = i+1;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());
    j++;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());
    j++;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());
    j++;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());
    j++;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());
    j++;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());
    j++;
    ROS_WARN("%F", yaml_node[j]["Angles"][i].as<float>());




  }  
};

// struct CostFunctor {
//   template <typename T> bool operator()(const T* const x, T* residual) const {
//     ;
//     return true;
//   }
// };

int main(int argc, char** argv)
{
  // CostFunctor cost_functor("sensor_data_1595950507.yaml");
  CostFunctor cost_functor("../config/2d_state_update_parameters.yaml");


  
  // double x = 0.5;
  // const double initial_x = x;

  // Problem problem;

  // CostFunction* cost_function =
  //     new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  // problem.AddResidualBlock(cost_function, NULL, &x);

  // Solver::Options options;
  // options.minimizer_progress_to_stdout = true;
  // Solver::Summary summary;
  // Solve(options, &problem, &summary);

  // std::cout << summary.BriefReport() << "\n";
  // std::cout << "x : " << initial_x
  //           << " -> " << x << "\n";
  return 0;
}
