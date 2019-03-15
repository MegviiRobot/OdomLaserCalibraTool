#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <rosbag/bag.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_odom_calib");
  ros::NodeHandle n("~");

  return 0;
}
