#ifndef CALIB_SYNCHRONIZER_H
#define CALIB_STNCHRONIZER_H

#endif

#include <scan_match.h>
#include <iostream>

class cSynchronizer {

public:

  /*!
   * \brief The sync_data struct. Used for
   * storing synchronized data.
   */
  struct sync_data {
    // Period
    double T;
    // Left and right wheel velocities
    double velocity_left;
    double velocity_right;
    // double velocity;
    // Scan matching estimate
    double scan_match_results[3];
    // Estimated rototranslation based on odometry params.
    double o[3];
    // Estimated disagreement  sm - est_sm
    double est_sm[3];
    double err_sm[3];
    // Other way to estimate disagreement:   l (+) s  - o (+) l
    double err[3];
    int mark_as_outlier;
  };

  cSynchronizer();

  /*!
   * \brief synchronizeLaserOdom. Member function used
   * for synchronizing timestamp between CSM results and
   * odometer.
   * \param odom_data
   * \param csm_results
   * \param sync_results
   */
  void synchronizeLaserOdom(const std::vector<messageIO::odometerData> &odom_data,
                            const std::vector<cScanMatch::csm_results> &csm_results,
                            std::vector<cSynchronizer::sync_data> &sync_results);

private:

};
