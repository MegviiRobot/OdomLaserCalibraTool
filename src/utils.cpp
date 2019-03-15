#include <utils.h>
#include <ros/ros.h>

std::string colouredString(std::string str, std::string colour, std::string option)
{
  double time_now = ros::Time::now().toSec();
  std::string time_string = std::to_string(time_now);
  return "[" + time_string + "]: " + option + colour + str + RESET;
}
