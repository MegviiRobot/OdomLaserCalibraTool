#include <scan_match.h>

using namespace CSM;

cScanMatch::cScanMatch() {
  // ROS_INFO("Setting CSM params");

  // sm_debug_write(1); // UNCOMMENT THIS FOR VISUALIZING DEBUG INFORMATION

  params.max_angular_correction_deg = 10.0;
  params.max_linear_correction = 0.05;
  params.max_iterations = 1000;
  params.epsilon_xy = 0.000001;
  params.epsilon_theta = 0.000001;
  params.max_correspondence_dist = 0.3;
  params.sigma = 0.010;
  params.use_corr_tricks = 1;
  params.restart = 0;
  params.restart_threshold_mean_error = 0.01;
  params.restart_dt = 1.0;
  params.restart_dtheta = 0.1;
  params.clustering_threshold = 0.25;
  params.orientation_neighbourhood = 20;
  params.use_point_to_line_distance = 1;
  params.do_alpha_test = 0;
  params.do_alpha_test_thresholdDeg = 20.0;
  params.outliers_maxPerc = 0.90;
  params.outliers_adaptive_order = 0.7;
  params.outliers_adaptive_mult = 2.0;
  params.do_visibility_test = 0;
  params.do_compute_covariance = 0;
  params.debug_verify_tricks = 0;
  params.use_ml_weights = 0;
  params.use_sigma_weights = 0;
  params.laser[0] = 0;
  params.laser[1] = 0;
  params.laser[2] = 0;
  params.first_guess[0] = 0;
  params.first_guess[1] = 0;
  params.first_guess[2] = 0;
  params.min_reading = 0;
  params.max_reading = 30.0;

}

void cScanMatch::scanToLDP(const std::vector<messageIO::laserScanData> &scan,
                           std::vector<LDP> &ldp) {

  int data_size = scan.size();

  for (int i = 0; i < data_size; i++)
  {
    LDP single_ldp;
    unsigned int n = scan[i].ranges.size();
    single_ldp = ld_alloc_new(n);
    for (int j = 0; j < n ; j++)
    {
      double r = scan[i].ranges[j];
      if ((r > scan[i].min_range) && (r < scan[i].max_range))
      {
        single_ldp->valid[j] = 1;
        single_ldp->readings[j] = r;
      }
      else
      {
        single_ldp->valid[j] = 0;
        single_ldp->readings[j] = -1;
      }

      single_ldp->theta[j] = scan[i].min_angle + j * scan[i].angle_increment;
      single_ldp->cluster[j] = -1;
    }

    single_ldp->min_theta = single_ldp->theta[0];
    single_ldp->max_theta = single_ldp->theta[n-1];

    single_ldp->odometry[0] = 0.0;
    single_ldp->odometry[1] = 0.0;
    single_ldp->odometry[2] = 0.0;

    single_ldp->true_pose[0] = 0.0;
    single_ldp->true_pose[1] = 0.0;
    single_ldp->true_pose[2] = 0.0;

    ldp.push_back(single_ldp);
  }

  return;
}

void cScanMatch::match(const std::vector<messageIO::laserScanData> &scan,
                       const std::vector<LDP> &ldp, std::vector<csm_odom> &scan_odom,
                       std::vector<csm_results> &match_results) {
  int ldp_size = ldp.size();
  int invalid_matches = 0;
  int scan_odom_it = 0;

  csm_odom first_odom;
  first_odom.x = -5.4056;
  first_odom.y = 13.6702;
  first_odom.theta = 3.10468;
  first_odom.nvalid = scan[0].ranges.size();
  first_odom.iterations = 0;
  first_odom.error = 0;
  first_odom.timestamp = scan[0].timestamp;

  scan_odom.push_back(first_odom);

  std::cout << '\n' << "Matching progress: ";

  for (int i = 1; i < ldp_size; i++)
  {
    // Free covariance;
    results.cov_x_m = 0;
    results.dx_dy1_m = 0;
    results.dx_dy2_m = 0;

    LDP ref_scan = ldp[scan_odom_it];
    LDP sens_scan = ldp[i];

    params.laser_ref = ref_scan;
    params.laser_sens = sens_scan;

    sm_icp(&params, &results);

    csm_odom last_odom;
    last_odom.timestamp = scan[i].timestamp;

    /* Transformation Composition
     *
     *  _        _      _       _     _                                         _
     * |   a_x   |     |   b_x   |   | a_x + b_x.cos(a_theta) - b_y.sin(a_theta) |
     * |   a_y   | [+] |   b_y   | = | a_y + b+x.sin(a_theta) + b_y.cos(a_theta) |
     * | a_theta |     | b_theta |   |           a_theta + b_theta               |
     *  -       -       -       -     -                                         -
     *
     */

    last_odom.x = results.x[0] + scan_odom.back().x * cos(results.x[2]) - scan_odom.back().y * sin(results.x[2]);
    last_odom.y = results.x[1] + scan_odom.back().x * sin(results.x[2]) + scan_odom.back().y * cos(results.x[2]);
    last_odom.theta = rad_fix(results.x[2], scan_odom.back().theta);
    last_odom.nvalid = results.nvalid;
    last_odom.iterations = results.iterations;
    last_odom.error = results.error;

    csm_results last_result;
    last_result.T = scan[i].timestamp.toSec() - scan[i-1].timestamp.toSec();
    last_result.start_t = scan[i-1].timestamp;
    last_result.end_t = scan[i].timestamp;
    last_result.scan_match_results[0] = results.x[0];
    last_result.scan_match_results[1] = results.x[1];
    last_result.scan_match_results[2] = results.x[2];
    match_results.push_back(last_result);

    if (results.valid == 1){
      scan_odom.push_back(last_odom);
      scan_odom_it ++;
    }
    else {
      invalid_matches ++;
    }
    int progress = ((float)i / (float)ldp_size) * 100;

    std::cout.width(3);
    std::cout << progress << "%";
    std::cout << "\b\b\b\b";
  }

  std::cout << '\n' << std::endl;
  std::cout << colouredString("Invaild matches: ", WHITE, REGULAR) << invalid_matches << std::endl;
  std::cout << colouredString("Vaild rate: ", WHITE, REGULAR) << (float)(1 - ((float)(invalid_matches) / (float)(ldp_size))) << std::endl;

  return;
}
