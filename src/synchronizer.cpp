#include <synchronizer.h>

cSynchronizer::cSynchronizer() {
  // ROS_INFO("Initing synchronizer...");
}

void cSynchronizer::synchronizeLaserOdom(const std::vector<messageIO::odometerData> &odom_data,
                                        const std::vector<cScanMatch::csm_results> &csm_results,
                                        std::vector<cSynchronizer::sync_data> &sync_results) {

  int odom_it = 0;
  ros::Time start_t_1;
  ros::Time start_t_2;
  ros::Time end_t_1;
  ros::Time end_t_2;
  double start_v_1_l;
  double start_v_1_r;
  double start_v_2_l;
  double start_v_2_r;
  double end_v_1_l;
  double end_v_1_r;
  double end_v_2_l;
  double end_v_2_r;

  std::cout << '\n' << "Synchronzing progress: ";

  for (int i = 5; i < csm_results.size(); i++)
  {
    // Find nearly timestamp for odometer.
    for (int j = odom_it; j < odom_data.size(); j++)
    {
      if ((odom_data[j].timestamp.toSec() < csm_results[i].start_t.toSec()) &&
          (odom_data[j+1].timestamp.toSec() > csm_results[i].start_t.toSec()))
      {
        start_t_1 = odom_data[j].timestamp;
        start_t_2 = odom_data[j+1].timestamp;
        start_v_1_l = odom_data[j].v_l;
        start_v_1_r = odom_data[j].v_r;
        start_v_2_l = odom_data[j+1].v_l;
        start_v_2_r = odom_data[j+1].v_r;
        odom_it = j;
        continue;
      }
      if (odom_data[j].timestamp.toSec() == csm_results[i].start_t.toSec())
      {
        start_t_1 = odom_data[j].timestamp;
        start_t_2 = odom_data[j].timestamp;
        start_v_1_l = odom_data[j].v_l;
        start_v_1_r = odom_data[j].v_r;
        start_v_2_l = odom_data[j].v_l;
        start_v_2_r = odom_data[j].v_r;
        odom_it = j;
        continue;
      }
      if ((odom_data[j].timestamp.toSec() < csm_results[i].end_t.toSec()) &&
          (odom_data[j+1].timestamp.toSec() > csm_results[i].end_t.toSec()))
      {
        end_t_1 = odom_data[j].timestamp;
        end_t_2 = odom_data[j+1].timestamp;
        end_v_1_l = odom_data[j].v_l;
        end_v_1_r = odom_data[j].v_r;
        end_v_2_l = odom_data[j+1].v_l;
        end_v_2_r = odom_data[j+1].v_r;
        odom_it = j;
        continue;
      }
      if (odom_data[j].timestamp.toSec() == csm_results[i].end_t.toSec())
      {
        end_t_1 = odom_data[j].timestamp;
        end_t_2 = odom_data[j].timestamp;
        end_v_1_l = odom_data[j].v_l;
        end_v_1_r = odom_data[j].v_r;
        end_v_2_l = odom_data[j].v_l;
        end_v_2_r = odom_data[j].v_r;
        odom_it = j;
        continue;
      }
    }

    float alpha = (csm_results[i].start_t.toSec() - start_t_1.toSec()) /
        (start_t_2.toSec() - start_t_1.toSec());

    double velocity_start_l = alpha * start_v_1_l + (1 - alpha) * start_v_2_l;
    double velocity_start_r = alpha * start_v_1_r + (1 - alpha) * start_v_2_r;

    float beta = (csm_results[i].end_t.toSec() - end_t_1.toSec()) /
        (end_t_2.toSec() - end_t_1.toSec());

    double velocity_end_l = beta * end_v_1_l + (1 - beta) * end_v_2_l;
    double velocity_end_r = beta * end_v_1_r + (1 - beta) * end_v_2_r;

    sync_data single_sync_data;
    single_sync_data.T = csm_results[i].T;
    single_sync_data.velocity_left = (velocity_start_l + velocity_end_l) / 2;
    single_sync_data.velocity_right = (velocity_start_r + velocity_end_r) / 2;
    single_sync_data.scan_match_results[0] = csm_results[i].scan_match_results[0];
    single_sync_data.scan_match_results[1] = csm_results[i].scan_match_results[1];
    single_sync_data.scan_match_results[2] = csm_results[i].scan_match_results[2];

    if(std::isnan(single_sync_data.velocity_left) || std::isnan(single_sync_data.velocity_right))
    {
      continue;
    }
    else sync_results.push_back(single_sync_data);

    start_t_1 = end_t_1;
    start_t_2 = end_t_2;
    start_v_1_l = end_v_1_l;
    start_v_1_r = end_v_1_r;
    start_v_2_l = end_v_2_l;
    start_v_2_r = end_v_2_r;

    int progress = ((float)i / (float)(csm_results.size())) * 100;

    std::cout.width(3);
    std::cout << progress << "%";
    std::cout << "\b\b\b\b";
  }
  std::cout << '\n' << std::endl;

  return;
}
