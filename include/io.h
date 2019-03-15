#ifndef CALIB_IO_H
#define CALIB_IO_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>

/*!
 * \brief The messageIO class, includes rosbag interface
 * and message type transform.
 */
class messageIO {

public:

  messageIO();

  ~messageIO(){

  }

  /*!
   * \brief The odometerData struct. Used for storing data
   * read from odometer.
   */
  struct odometerData {
    ros::Time timestamp;
    float x;
    float y;
    float theta;
    Eigen::Vector3f linear_v;
    Eigen::Vector3f rotate_v;
    double v_l;
    double v_r;
  };

  /*!
   * \brief The laserScanData struct. Used for storing data
   * read from LiDAR.
   */
  struct laserScanData {
    ros::Time timestamp;
    float time_increment;
    float scan_time;
    float max_range;
    float min_range;
    float angle_increment;
    float min_angle;
    float max_angle;
    std::vector<float> ranges;
  };

  /*!
   * \brief readDataFromBag. Member function used for
   * reading data from a ROS bag.
   * \param bag_name. Input as a string.
   * \param laser_topic_name. Input as a string.
   * \param odom_topic_name. Input as a string.
   * \param odom_data. Vector of odometer data
   * transformed from ROS message.
   * \param laser_data. Vector of laser data
   * transformed from ROS message.
   */
  void readDataFromBag(const std::string &bag_name, const std::string &laser_topic_name, const std::string &odom_topic_name,
            std::vector<odometerData> &odom_data, std::vector<laserScanData> &laser_data);
private:

  /*!
   * \brief getTheta. Private member function used
   * for get Yaw angle from a quaternion.
   * \param q. Quaternion, input as a geometry_msgs
   * type quaternion.
   * \return return by Yaw angle as a float.
   */
  float getTheta(const geometry_msgs::Quaternion &q)
  {
//    Eigen::Quaternionf eigen_q(q.w, q.x, q.y, q.z);
//    Eigen::Matrix3f r = Eigen::Matrix3f::Identity();
//    r = eigen_q.toRotationMatrix();
//    Eigen::Vector3f e = r.eulerAngles(2,1,0);
//    return e(0);
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    float yaw = atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

};

#endif
