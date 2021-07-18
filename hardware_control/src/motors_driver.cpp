#include "motors_driver.hpp"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/actuator_state_interface.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  CarRobot robot;
  controller_manager::ControllerManager cm(&robot,nh);


  ros::AsyncSpinner spinner(2, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read();
     cm.update(ts, d);
     robot.write();
     rate.sleep();
  }

  spinner.stop();

  return 0;
}
