#include "ceres/ceres.h"
#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "ros/ros.h"
#include <string>
#include <cmath>

// cost functor used by ceres
struct CostFunctor
{
  // cost functor constructor
  CostFunctor(double theta_1, double theta_2, double true_ee_position_x, double true_ee_position_y,
              double link_1_length, double link_2_length)
    : theta_1_(theta_1)
    , theta_2_(theta_2)
    , true_ee_position_x_(true_ee_position_x)
    , true_ee_position_y_(true_ee_position_y)
    // link lengths are constants, currently hardcoded in main
    , link_1_length_(link_1_length)
    , link_2_length_(link_2_length)
  {
  }

  // templated operator to calculate residual, which is squared norm of the difference between the true ee position and
  // the ee position calculated with error
  template <typename T>
  bool operator()(const T* const offset_1, const T* const offset_2, T* residual) const
  {
    // these 2 variables used to make calculation understandable
    T offset_position_y = (T(link_1_length_) * T(sin(T(theta_1_) + offset_1[0]))) +
                          (T(link_2_length_) * T(sin((T(theta_1_) + offset_1[0]) + (T(theta_2_) + offset_2[0]))));
    T offset_position_x = (T(link_1_length_) * T(cos((T(theta_1_) + offset_1[0])))) +
                          (T(link_2_length_) * T(cos((T(theta_1_) + offset_1[0]) + (T(theta_2_) + offset_2[0]))));
    // actual residual
    residual[0] = (T(true_ee_position_y_) - offset_position_y) * (T(true_ee_position_y_) - offset_position_y) +
                  (T(true_ee_position_x_) - offset_position_x) * (T(true_ee_position_x_) - offset_position_x);
    return true;
  }

private:
  const double theta_1_, theta_2_, true_ee_position_x_, true_ee_position_y_, link_1_length_, link_2_length_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrator");

  ros::NodeHandle nh;

  std::string filepath = ros::package::getPath("robot_calibration");

  std::string filename;
  nh.getParam("/calibrator/filename", filename);

  if (filepath != "")
  {
    filepath = filepath + "/sensor_data/" + filename;
  }
  else
  {
    ros::shutdown();
  }

  // loading yaml file
  YAML::Node yaml_node = YAML::LoadFile(filepath);

  // size of file indicating number of data points obtained
  int num_observations = yaml_node.size();

  // hardcoded link lengths
  double link_1_length = 1.8;
  double link_2_length = 1;

  // declaration of arrays used to store fields separately
  double theta_1_arr[num_observations], theta_2_arr[num_observations], true_ee_position_x[num_observations],
      true_ee_position_y[num_observations];

  // populating arrays from loaded yaml file
  int count = 0;
  for (std::size_t i = 0; i < num_observations; i++)
  {
    theta_1_arr[count] = yaml_node[i]["Angles"][0].as<float>();
    theta_2_arr[count] = yaml_node[i]["Angles"][1].as<float>();
    true_ee_position_x[count] = yaml_node[i]["End Effector Position"][0].as<float>();
    true_ee_position_y[count] = yaml_node[i]["End Effector Position"][1].as<float>();
    count++;
  }

  // initial values for offsets, to be altered by ceres
  double offset_1 = 0;
  double offset_2 = 0;

  ceres::Problem problem;

  // adding a residual block to the problem for each observation
  for (int i = 0; i < num_observations; i++)
  {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(new CostFunctor(
        theta_1_arr[i], theta_2_arr[i], true_ee_position_x[i], true_ee_position_y[i], link_1_length, link_2_length));
    problem.AddResidualBlock(cost_function, NULL, &offset_1, &offset_2);
  }

  // configuring options
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // print final output
  // offsets are negated so that final answer reflects offset applied, rather than offset needed to correct sensor
  std::cout << summary.BriefReport() << std::endl;
  std::cout << "Initial o1: " << 0.0 << " o2: " << 0.0 << "\n";
  std::cout << "Final   o1: " << -offset_1 << " o2: " << -offset_2 << "\n";

  return 0;
}
