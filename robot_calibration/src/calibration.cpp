#include "ceres/ceres.h"
#include "yaml-cpp/yaml.h"
#include "boost/filesystem.hpp"
#include "ros/ros.h"
#include <string>
#include <Eigen/Dense>


// using ceres::AutoDiffCostFunction;
// using ceres::CostFunction;
// using ceres::Problem;
// using ceres::Solver;
// using ceres::Solve;


  // ee_position = {(link_1_length * cos(theta_1)) + (link_2_length * cos(theta_1 + theta_2)) , (link_1_length * sin(theta_1)) + (link_2_length * sin(theta_1 + theta_2))};




struct CostFunctor {
  template <typename T> bool operator()(const T* const theta_1, const T* const theta_2, const T* const link_1_length, const T* const link_2_length, const T* const offset_1, const T* const offset_2, T* residual) const {
    residual[0] = 4; 
    return true;
  }
};

int main(int argc, char** argv)
{
  YAML::Node yaml_node = YAML::LoadFile("/home/jad/catkin_ws/src/Robot-Calibration-Deep-Dives-DP/robot_calibration/sensor_data/fake_testing.yaml");
    
    for(std::size_t i=0; i<yaml_node.size(); i++)
    {
      for(std::size_t j=0; j<2; j++)
      {
        ROS_WARN("%f", yaml_node[i]["Angles"][j].as<float>());
      }
      for(std::size_t j=0; j<2; j++)
      {
        ROS_WARN("%f", yaml_node[i]["End Effector Position"][j].as<float>());
      }
    }

  


  
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
