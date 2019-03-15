#include <io.h>
#include <utils.h>

messageIO::messageIO()
{
  // ROS_INFO("Initing IO...");
}

void messageIO::readDataFromBag(const std::string &bag_name, const std::string &laser_topic_name, const std::string &odom_topic_name,
          std::vector<odometerData> &odom_data, std::vector<laserScanData> &laser_data)
{
  rosbag::Bag bag;
  std::cout << colouredString("Opening bag...", YELLOW, REGULAR) << std::endl;

  try{
    bag.open(bag_name, rosbag::bagmode::Read);
  }
  catch (std::exception& e) {
    std::cout << colouredString("ERROR!", RED, BOLD) << std::endl;
  }

  std::cout << colouredString("DONE!", GREEN, REGULAR) << std::endl;
  std::cout << colouredString("Quering topics bag...", YELLOW, REGULAR) << std::endl;

  std::vector<std::string> topics;
  topics.push_back(laser_topic_name);
  topics.push_back(odom_topic_name);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // std::cout << colouredString("Reading bag data...", YELLOW, REGULAR) << std::endl;

  for (rosbag::MessageInstance const m: view) {
    sensor_msgs::LaserScanConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
    if (scan != NULL) {
      laserScanData tmp;
      tmp.ranges = scan->ranges;
      tmp.scan_time = scan->scan_time;
      tmp.time_increment = scan->time_increment;
      tmp.timestamp = scan->header.stamp;
      tmp.angle_increment = scan->angle_increment;
      tmp.max_angle = scan->angle_max;
      tmp.min_angle = scan->angle_min;
      tmp.max_range = scan->range_max;
      tmp.min_range = scan->range_min;
      laser_data.push_back(tmp);
    }

//    nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();
//    if (odom != NULL) {
//      odometerData tmp;
//      tmp.timestamp = odom->header.stamp;
//      double r_l = 0.4;
//      double r_r = 0.4;
//      double b = 0.7;
//      double v = odom->twist.twist.linear.x;
//      double omega = odom->twist.twist.angular.z;
//      tmp.v_l = (v / r_l) - ((omega * b) / (2 * r_l));
//      tmp.v_r = (v / r_r) + ((omega * b) / (2 * r_r));
//      odom_data.push_back(tmp);
//    }

//    sensor_msgs::JointStateConstPtr odom = m.instantiate<sensor_msgs::JointState>();
//    if (odom != NULL) {
//      odometerData tmp;
//      tmp.timestamp = odom->header.stamp;
//      tmp.v_l = odom->velocity[0];
//      tmp.v_r = odom->velocity[1];
//      odom_data.push_back(tmp);
//    }
//
    geometry_msgs::Vector3StampedConstPtr odom = m.instantiate<geometry_msgs::Vector3Stamped>();
    if (odom != NULL) {
      odometerData tmp;
      tmp.timestamp = odom->header.stamp;
      tmp.v_l = odom->vector.x;
      tmp.v_r = odom->vector.y;
      odom_data.push_back(tmp);
    }
  }

  // std::cout << "laser size: " << laser_data.size() << '\n' << "odom size: " << odom_data.size() << std::endl;

  std::cout << colouredString("Data reading finished!", GREEN, REGULAR) << std::endl;

  return;

}
