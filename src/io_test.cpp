#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <io.h>
#include <scan_match.h>
#include <solver.h>

messageIO dataIO;
cScanMatch cScan;
cSynchronizer cSync;
cSolver cSolve;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calib_io");
  ros::NodeHandle n("~");

//  std::string file_path = "/home/wangzhijie/Downloads/dataset/";
//  std::string bagname = "2018-12-28-15-16-07";
  std::string file_path;
  std::string bagname;
  std::string laser_topic;
  std::string odom_topic;

  ros::param::get("~file_path", file_path);
  ros::param::get("~bagname", bagname);
  ros::param::get("~laser_topic", laser_topic);
  ros::param::get("~odom_topic", odom_topic);

  std::string bagfile = file_path + bagname + ".bag";
  std::string resultfile = file_path + bagname + "plicp_results.txt";
  std::string syncfile = file_path + bagname + "sync_results.txt";
  std::string smfile = file_path + bagname + "scan_match_results.txt";
  std::string odomfile = file_path + bagname + "odom.txt";
//  std::string laser_topic = "/scan";
//  std::string odom_topic = "/odom";
  std::vector<messageIO::laserScanData> laser_data(0);
  std::vector<messageIO::odometerData> odom_data(0);

  dataIO.readDataFromBag(bagfile, laser_topic, odom_topic, odom_data, laser_data);

  std::cout << colouredString("Hi!", WHITE, REGULAR) << std::endl;
  std::cout << colouredString("laser size: ", WHITE, REGULAR) << laser_data.size() << '\n'
            << colouredString("odom size: ", WHITE, REGULAR) << odom_data.size() << std::endl;

  std::cout << colouredString("Begin CSM!", GREEN, REGULAR) << std::endl;
  std::vector<cScanMatch::csm_odom> scan_odom(0);
  std::vector<LDP> ldp(0);
  std::vector<cScanMatch::csm_results> match_results(0);

  std::cout << colouredString("Converting scan to csm type...", YELLOW, REGULAR) << std::endl;
  cScan.scanToLDP(laser_data, ldp);
  std::cout << colouredString("Converting finished!", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("LDP numbers: ", WHITE, REGULAR) << ldp.size() << std::endl;

  std::cout << colouredString("Canonical scan matching...", YELLOW, REGULAR) << std::endl;
  cScan.match(laser_data, ldp, scan_odom, match_results);
  std::cout << colouredString("Matching finished!", GREEN, REGULAR) << std::endl;

  if (scan_odom.size() == 0) {
    std::cout << colouredString("ERROR! No matching results!", RED, BOLD) << std::endl;
  }

  std::cout << colouredString("Writing results...", YELLOW, REGULAR) << std::endl;

  std::ofstream fout(resultfile);
  for (int i = 0; i < scan_odom.size(); i++)
  {
    fout << scan_odom[i].timestamp << ' ' << scan_odom[i].x << ' ' << scan_odom[i].y
         << ' ' << scan_odom[i].theta << '\n';
  }

  std::ofstream fout3(smfile);
  for (int i = 0; i < match_results.size(); i++)
  {
    fout3 << match_results[i].start_t << ' ' << match_results[i].end_t << ' '
          << match_results[i].T << ' ' << match_results[i].scan_match_results[0] << ' '
          << match_results[i].scan_match_results[1] << ' '
          << match_results[i].scan_match_results[2] << '\n';
  }

  std::ofstream fout4(odomfile);
  for (int i = 0; i < odom_data.size(); i ++)
  {
    fout4 << odom_data[i].timestamp << ' ' << odom_data[i].v_l << ' '
          << odom_data[i].v_r << '\n';
  }

  std::cout << colouredString("Saving results...", YELLOW, REGULAR) << std::endl;
  fout << std::flush;
  fout.close();
  fout3 << std::flush;
  fout3.close();
  fout4 << std::flush;
  fout4.close();

  std::cout << colouredString("Results saved!", GREEN, REGULAR) << std::endl;

  std::cout << colouredString("Start synchronzing...", YELLOW, REGULAR) << std::endl;
  std::vector<cSynchronizer::sync_data> sync_results;
  cSync.synchronizeLaserOdom(odom_data, match_results, sync_results);
  std::cout << colouredString("Sync finished!", GREEN, REGULAR) << std::endl;

  std::ofstream fout2(syncfile);
  for (int i = 0; i < sync_results.size(); i++)
  {
    fout2 << sync_results[i].T << ' ' << sync_results[i].velocity_left << ' '
          << sync_results[i].velocity_right << ' '
          << sync_results[i].scan_match_results[0] << ' '
          << sync_results[i].scan_match_results[1] << ' '
          << sync_results[i].scan_match_results[2] << '\n';
  }
  fout2 << std::flush;
  fout2.close();

  std::cout << colouredString("Sync results saved!", GREEN, REGULAR) << std::endl;

  cSolve.calib(sync_results, 4);

  ros::spin();
  return 0;
}
