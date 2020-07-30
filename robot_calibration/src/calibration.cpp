#include "ceres/ceres.h"
#include "yaml-cpp/yaml.h"
#include "boost/filesystem.hpp"
#include "ros/package.h"
#include "ros/ros.h"
#include <string>
#include <Eigen/Dense>
#include <iostream>


struct CostFunctor {
  CostFunctor(double theta_1, double theta_2, double true_ee_position_x, double true_ee_position_y, double link_1_length, double link_2_length)
    : theta_1_(theta_1), theta_2_(theta_2), true_ee_position_x_(true_ee_position_x), true_ee_position_y_(true_ee_position_y), link_1_length_(link_1_length), link_2_length_(link_2_length) {}
  
  template <typename T> bool operator()(const T* const offset_1, const T* const offset_2, T* residual) const {
   
    // Eigen::Vector2f true_ee_position(T(true_ee_position_x_), T(true_ee_position_y_));
    Eigen::Vector2f true_ee_position(true_ee_position_x_, true_ee_position_y_);
    Eigen::Vector2f calculated_ee_position(((link_1_length_ * cos((theta_1_ + offset_1[0]))) + (link_2_length_ * cos((theta_1_ + offset_1[0]) + (theta_2_ + offset_2[0])))), ((link_1_length_ * sin(theta_1_ + offset_1[0])) + (link_2_length_ * sin((theta_1_ + offset_1[0]) + (theta_2_ + offset_2[0])))));
    residual[0] = T((true_ee_position - calculated_ee_position).squaredNorm());
    return true;
  }
  private:
    const double theta_1_, theta_2_, true_ee_position_x_, true_ee_position_y_, link_1_length_, link_2_length_;

};

int main(int argc, char** argv)
{
  YAML::Node yaml_node = YAML::LoadFile("/home/jad/catkin_ws/src/Robot-Calibration-Deep-Dives-DP/robot_calibration/sensor_data/fake_testing.yaml");

  int num_observations = yaml_node.size();

  double link_1_length = 1.8;
  double link_2_length = 1;

  double theta_1_arr[num_observations], theta_2_arr[num_observations], true_ee_position_x[num_observations], true_ee_position_y[num_observations];

  int count = 0;  
  for(std::size_t i=0; i<num_observations; i++)
  {
    theta_1_arr[count] = yaml_node[i]["Angles"][0].as<float>();
    theta_2_arr[count] = yaml_node[i]["Angles"][1].as<float>();
    true_ee_position_x[count] = yaml_node[i]["End Effector Position"][0].as<float>();
    true_ee_position_y[count] = yaml_node[i]["End Effector Position"][1].as<float>();
    count++;
  }

  // double true_ee_position_x_ = 0.9946222072248587;
  // double true_ee_position_y_ = 1.153743244584414;
  // double theta_1_ = -5.968654341404178;
  // double theta_2_ = -4.136983034538698;
  // double offset_1 = 0.044;
  // double offset_2 = 0.013;
  // double link_1_length_ = 1.8;
  // double link_2_length_ = 1;

  // Eigen::Vector2f true_ee_position(true_ee_position_x_, true_ee_position_y_);
  // Eigen::Vector2f calculated_ee_position(((link_1_length_ * cos((theta_1_ + offset_1))) + (link_2_length_ * cos((theta_1_ + offset_1) + (theta_2_ + offset_2)))), ((link_1_length_ * sin(theta_1_ + offset_1)) + (link_2_length_ * sin((theta_1_ + offset_1) + (theta_2_ + offset_2))))  );
  // double test = (true_ee_position - calculated_ee_position).squaredNorm();

  // std::cout << test << std::endl;

  double offset_1 = 0;
  double offset_2 = 0;

  ceres::Problem problem;

  for(int i=0; i<num_observations; i++)
  {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(new CostFunctor(theta_1_arr[i], theta_2_arr[i], true_ee_position_x[i], true_ee_position_x[i], link_1_length, link_2_length));
    problem.AddResidualBlock(cost_function, NULL, &offset_1, &offset_2);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);


  std::cout << summary.BriefReport() << std::endl;
  std::cout << "Initial o1: " << 0.0 << " o2: " << 0.0 << "\n";
  std::cout << "Final   o2: " << offset_1 << " o2: " << offset_2 << "\n";

  return 0;
}
