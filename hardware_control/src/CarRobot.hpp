#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <iostream>

class CarRobot : public hardware_interface::RobotHW
{
public:
  CarRobot()
 {

   hardware_interface::JointStateHandle state_handle_drive("rear_wheel_joint", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_drive);
   hardware_interface::JointStateHandle state_handle_steering("front_steer_joint", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_steering);

   hardware_interface::JointHandle vel_handle_drive(jnt_state_interface.getHandle("rear_wheel_joint"), &vel[0]);
   hardware_interface::JointHandle pos_handle_steering(jnt_state_interface.getHandle("front_steer_joint"), &pos[1]);

   jnt_pos_interface.registerHandle(pos_handle_steering);
   jnt_vel_interface.registerHandle(vel_handle_drive);

   registerInterface(&jnt_state_interface);
   registerInterface(&jnt_pos_interface);
   registerInterface(&jnt_vel_interface);
}

  virtual ~CarRobot(){}

  void write()
  {
	  std::cout<<"write "<<"  "<< vel[0]<<" "<< vel[1] << std::endl;
	  std::cout<<"write "<<"  "<< pos[0]<<" "<< pos[1] << std::endl;
	  std::cout<< std::endl;
  }

  void read()
  {
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;

  double pos[2] = {0, 0};
  double vel[2] = {0, 0};
  double eff[2] = {0, 0};

};
