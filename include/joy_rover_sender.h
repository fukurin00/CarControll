#ifndef JOY_VEHICLE_CND_SENDER_H_INCLUDED
#define JOY_VEHICLE_CND_SENDER_H_INCLUDED

#include "ros/ros.h"

#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <boost/optional.hpp>
#include <boost/thread.hpp>

class joy_vehicle_cmd_sender
{
public:
    joy_vehicle_cmd_sender();
    ~joy_vehicle_cmd_sender();
    void run();
    enum class GearMode{Nutral, First, Second, Third, Fourth, Fifth, Sixth, Back};
private:
    GearMode gear_mode;
    boost::optional<sensor_msgs::Joy> joy_cmd_;
    boost::optional<geometry_msgs::Twist> rover_odo_;
    ros::NodeHandle nh_;

    
    ros::Subscriber joy_sub_;
    ros::Subscriber odo_sub_;
    ros::Publisher vehicle_cmd_pub_;

    geometry_msgs::Twist get_twist();
    void publish_vehicle_cmd_();
    void joy_callback_(sensor_msgs::Joy msg);
    void odo_callback_(geometry_msgs::Twist msg);
    
    
};
#endif  //JOY_VEHICLE_CND_SENDER_H_INCLUDED
