#include "motors_driver.hpp"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "steerbot");
  ros::NodeHandle nh;

  CarRobot robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ROS_WARN_STREAM("LOADED CONTROLLERS");
  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
