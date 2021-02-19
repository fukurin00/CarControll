#include "joy_rover_sender.h"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "joy_rover");
  joy_vehicle_cmd_sender sender;
  sender.run();
  ros::spin();
  return 0;
}