#ifndef CALIB_SCAN_MATCH_H
#define CALIB_SCAN_MATCH_H

#include <utils.h>
#include <io.h>
#include <csm/csm_all.h>

using namespace CSM;

/*!
 * \brief The cScanMatch class, includes CSM
 * interface and data type transformation.
 */
class cScanMatch{

public:

  /*!
   * \brief cScanMatch. Construct function,
   * includes CSM parametes initialization.
   */
  cScanMatch();

  /*!
   * \brief The csm_odom struct. Used for
   * storing odom from CSM.
   */
  struct csm_odom {
    ros::Time timestamp;
    float x;
    float y;
    float theta;
    int iterations;
    int nvalid;
    double error;
  };

  /*!
   * \brief The csm_results struct. Used
   * for storing CSM results each interval.
   */
  struct csm_results {
    ros::Time start_t;
    ros::Time end_t;
    double T;
    double scan_match_results[3];
  };

  /*!
   * \brief scanToLDP. Member function used
   * for transforming scan data to LDP type.
   * \param scan. Laser scan data, input as a
   * vector of messageIO::laserScanData.
   * \param ldp. LDP data, input as a vector
   * of CSM::LDP.
   */
  void scanToLDP(const std::vector<messageIO::laserScanData> &scan, std::vector<LDP> &ldp);

  /*!
   * \brief match. Member function used for
   * obtaining CSM results.
   * \param scan. Laser scan data, input as a
   * vector of messageIO::laserScanData.
   * \param ldp. LDP data, input as a vector
   * of CSM::LDP.
   * \param scan_odom. Odom from CSM.
   * \param match_results. Matching results
   * each interval.
   */
  void match(const std::vector<messageIO::laserScanData> &scan, const std::vector<LDP> &ldp,
             std::vector<csm_odom> &scan_odom, std::vector<csm_results> &match_results);

private:

  sm_params params;
  sm_result results;

  /*!
   * \brief rad_fix. Private member function used
   * for fix radian range.
   * \param rad_a. Former radian, input as a float.
   * \param rad_b. New radian, input as a float.
   * \return
   */
  float rad_fix(const float &rad_a, const float &rad_b)
  {
    if ((rad_a + rad_b) > 3.14159)
    {
      return (6.28319 - (rad_a + rad_b));
    }
    else if((rad_a + rad_b) < -3.14159)
    {
      return rad_a + rad_b + 6.28319;
    }
    else return rad_a + rad_b;
  }

};


#endif
