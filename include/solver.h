#ifndef CALIB_SOLVER_H
#define CALIB_SOLVER_H

#include <synchronizer.h>

class cSolver{

public:

  cSolver();

  struct solver_params {
    int mode;

    double max_cond_number;

    int outliers_iterations;
    double outliers_percentage;
  };

  struct calib_result {
    double radius_l, radius_r;
    double axle;

    /** Laser pose */
    double l[3];
  };

  bool solve(const std::vector<cSynchronizer::sync_data> &calib_data, int mode, double max_cond_number, struct calib_result& res);

  void calib(std::vector<cSynchronizer::sync_data> &calib_data, int outliers_iterations);

private:

  Eigen::VectorXd full_calibration_min(const Eigen::MatrixXd &M);

  Eigen::VectorXd numeric_calibration(const Eigen::MatrixXd &H);

  double calculate_error(const Eigen::VectorXd &x, const Eigen::MatrixXd &M);

  Eigen::VectorXd x_given_lambda(const Eigen::MatrixXd &M, const double &lambda, const Eigen::MatrixXd &W);

  void compute_disagreement(cSynchronizer::sync_data &calib_data, const struct calib_result &res);

  void estimate_noise(std::vector<cSynchronizer::sync_data> &calib_data, const struct calib_result &res,
                      double &std_x, double &std_y, double &std_th);

  double calculate_sd(const double array[], const int s, const int e);

  Eigen::MatrixXd compute_fim(const std::vector<cSynchronizer::sync_data> &calib_data, const struct calib_result &res,
                   const Eigen::Matrix3d &inf_sm);

};

#endif
