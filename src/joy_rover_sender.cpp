#include "joy_rover_sender.h"

enum ControlMode {ManualJwStick, ManualJoyStick, Auto, Hold};


ControlMode control_mode;

joy_vehicle_cmd_sender::joy_vehicle_cmd_sender()
{
  joy_cmd_ = boost::none;
  rover_odo_ = boost::none;

  vehicle_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/rover_twist", 10);
  joy_sub_ = nh_.subscribe("/joy", 10, &joy_vehicle_cmd_sender::joy_callback_, this);
  odo_sub_ = nh_.subscribe("/rover_odo", 10, &joy_vehicle_cmd_sender::odo_callback_, this);
  
  control_mode = ManualJoyStick;
  gear_mode = GearMode::First;
}

joy_vehicle_cmd_sender::~joy_vehicle_cmd_sender()
{
}

void joy_vehicle_cmd_sender::run()
{
  boost::thread publish_vehicle_cmd_thread_(&joy_vehicle_cmd_sender::publish_vehicle_cmd_, this);
  return;
}

geometry_msgs::Twist joy_vehicle_cmd_sender::get_twist()
{
  /*
    axes = [Handle(r:-1, l:1),Accel(-1,1),Crach(-1,1),Brake(-1,1),yoko,tate]
    buttuns = [left, right, triangle, squrere, circle, cross,,,]
  */

  double v;      

  if (gear_mode==GearMode::First)
  {
    v = rover_odo_->linear.x  + joy_cmd_->axes[1]+1;
    if (joy_cmd_->axes[3] > -1) //brake on
    {
      if (v > (joy_cmd_->axes[3]+1))
      {
        v -= (joy_cmd_->axes[3]+1);
      }else{
        v = 0;
      }
    }
  }
  else if (gear_mode == GearMode::Back)
  {
    v = rover_odo_->linear.x  - joy_cmd_->axes[1]+1;
    if (joy_cmd_->axes[3] > -1) //brake on
    {
      if (v < -1*(joy_cmd_->axes[3]+1))
      {
        v += (joy_cmd_->axes[3]+1);
      }
      else
      {
        v = 0;
      }
    }
  }
  
  double w = joy_cmd_->axes[0];
  double lr = joy_cmd_ -> axes[4];

  double linearX = 1.25*v;
  double linearY = lr*1.25;
  double angularZ = w*1.25;

  geometry_msgs::Twist vel;

  vel.linear.x = linearX;
  vel.linear.y = linearY;
  vel.angular.z = angularZ;
  return vel;
}

void joy_vehicle_cmd_sender::publish_vehicle_cmd_()
{
  ros::Rate rate(30);
  while (ros::ok())
  {
    //*** IMPORTANT: DS4'S BUTTON ASSIGNS ARE FOR MODEL "CUH-ZCT2J" ***//
    if (joy_cmd_ && control_mode==ManualJoyStick)  // R2：Enable
    {
      /*
        axes = [Handle(r:-1, l:1),Accel(-1,1),Crach(-1,1),Brake(-1,1),yoko,tate]
        buttuns = [left, right, triangle, squrere, circle, cross,,,]
      */
      geometry_msgs::Twist vel;
      vel = get_twist();
      vehicle_cmd_pub_.publish(vel);
      ROS_INFO("liner_x: %f, angular_z: %f", vel.linear.x, vel.angular.z);      
    }
    rate.sleep();
  }
  return;
}

void joy_vehicle_cmd_sender::odo_callback_(geometry_msgs::Twist msg)
{
  rover_odo_ = msg;
  return; 
}

void joy_vehicle_cmd_sender::joy_callback_(sensor_msgs::Joy msg)
{
  /*
    axes = [Handle(r:-1, l:1),Accel(-1,1),Crach(-1,1),Brake(-1,1),yoko,tate]
    buttuns = [left, right, triangle, squrere, circle, cross,,,]
  */
  if(msg.buttons[5] == 1) 
  { //cross button
    control_mode = ManualJoyStick;
  } 
  else if(msg.buttons[4] == 1)
  {
    control_mode = Auto; //circle button
  }

  if(msg.axes[2] > -0.5 && msg.buttons[0]==1)
  {
    gear_mode = GearMode::First;
  } 
  else if(msg.axes[2] > -0.5 && msg.buttons[1]==1)
  {
    gear_mode = GearMode::Back;
  }

  joy_cmd_ = msg;
  return;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "joy_rover");
  joy_vehicle_cmd_sender sender;
  sender.run();
  ros::spin();
  return 0;
}