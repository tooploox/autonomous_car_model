#include "Python.h"
#include "py_driver.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>

#include <limits>
#include <cmath>

#define PI 3.1415926535897
#define SERVO_DEG_OFFSET 90

class CarRobot : public hardware_interface::RobotHW
{
public:
  CarRobot()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &CarRobot::startCallback, this))
  , stop_srv_(nh_.advertiseService("stop", &CarRobot::stopCallback, this))
  {

    // Intialize raw data
    this->cleanUp();
    this->getJointNames(nh_);
    this->registerHardwareInterfaces();

    nh_.getParam(nh_.getNamespace() + "/ackermann_steering_controller/wheel_radius", wheel_radius_);
    nh_.getParam(nh_.getNamespace() + "/ackermann_steering_controller/linear/x/max_velocity", max_linear_velocity_);
    max_wheel_speed_ = max_linear_velocity_;

    nh_.getParam(nh_.getNamespace() + "/robot_info/max_steering_angle", max_steering_angle_);
    nh_.getParam(nh_.getNamespace() + "/robot_info/max_wheel_speed", max_wheel_speed_);
    nh_.getParam(nh_.getNamespace() + "/robot_info/max_reverse_wheel_speed", max_reverse_wheel_speed_);



    if (PyImport_AppendInittab("py_driver", PyInit_py_driver) == -1) {
        fprintf(stderr, "Error: could not extend in-built modules table\n");
        exit(1);
   }

   Py_Initialize();
   PyImport_ImportModule("py_driver");
   py_car_robot = instantiatePyCarRobot();

   if (py_car_robot == nullptr){
    exit(1);
   }

  }

  PyObject *py_car_robot = nullptr;
  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read()
  {
    /*
    TODO: Add support for motor encoder and read actual speed.
      There is no option to read steering servo position but as it's
      a closed loop we may assume that the position is the same as requested.
    */
  }

  void write()
  {
    if (running_)
    {
      // wheels
      rear_wheel_jnt_pos_ += rear_wheel_jnt_vel_*getPeriod().toSec();
      rear_wheel_jnt_vel_ = impose_speed_limit(rear_wheel_jnt_vel_cmd_, max_wheel_speed_, max_reverse_wheel_speed_);

      // steers
      front_steer_jnt_pos_ = impose_angle_limit(front_steer_jnt_pos_cmd_, max_steering_angle_);

      //Add offset at the PWM controller expects angles in 0-180 range
      set_angle(py_car_robot, radians2degrees(front_steer_jnt_pos_) + SERVO_DEG_OFFSET);
      set_throttle(py_car_robot, speed2throttle(rear_wheel_jnt_vel_, max_wheel_speed_, max_reverse_wheel_speed_));

      ROS_INFO_STREAM("Wheel pos: " << rear_wheel_jnt_pos_);
      ROS_INFO_STREAM("vel: " << rear_wheel_jnt_vel_);
      ROS_INFO_STREAM("pos: " << front_steer_jnt_pos_);
      ROS_INFO_STREAM("Throttle: " << speed2throttle(rear_wheel_jnt_vel_, max_wheel_speed_, max_reverse_wheel_speed_));
      ROS_INFO_STREAM("Steer angle: " << radians2degrees(front_steer_jnt_pos_));

    }
    else
    {
      // wheels
      rear_wheel_jnt_pos_= std::numeric_limits<double>::quiet_NaN();
      rear_wheel_jnt_vel_= std::numeric_limits<double>::quiet_NaN();

      // steers
      front_steer_jnt_pos_= std::numeric_limits<double>::quiet_NaN();
      front_steer_jnt_vel_= std::numeric_limits<double>::quiet_NaN();
    }
  }

  bool startCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    ROS_INFO_STREAM("running_ = " << running_ << ".");
    running_ = true;
    return true;
  }

  bool stopCallback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    ROS_INFO_STREAM("running_ = " << running_ << ".");
    running_ = false;
    return true;
  }

private:

  void cleanUp()
  {
    // wheel
    //-- wheel joint names
    rear_wheel_jnt_name_.empty();
    //-- actual rear wheel joint
    rear_wheel_jnt_pos_ = 0;
    rear_wheel_jnt_vel_ = 0;
    rear_wheel_jnt_eff_ = 0;
    rear_wheel_jnt_vel_cmd_ = 0;


    // steer
    //-- steer joint names
    front_steer_jnt_name_.empty();
    //-- front steer joint
    front_steer_jnt_pos_ = 0;
    front_steer_jnt_vel_ = 0;
    front_steer_jnt_eff_ = 0;
    front_steer_jnt_pos_cmd_ = 0;

  }

  void getJointNames(ros::NodeHandle &_nh)
  {
    this->getWheelJointNames(_nh);
    this->getSteerJointNames(_nh);
  }

  void getWheelJointNames(ros::NodeHandle &_nh)
  {
    // wheel joint to get linear command
    _nh.getParam(nh_.getNamespace() + "/ackermann_steering_controller/rear_wheel", rear_wheel_jnt_name_);
    ROS_WARN_STREAM("REAR wheel" << rear_wheel_jnt_name_);

  }

  void getSteerJointNames(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(nh_.getNamespace() + "/ackermann_steering_controller/front_steer", front_steer_jnt_name_);

  }

  void registerHardwareInterfaces()
  {
    this->registerSteerInterface();
    this->registerWheelInterface();

    // register mapped interface to ros_control
    registerInterface(&jnt_state_interface_);
    registerInterface(&rear_wheel_jnt_vel_cmd_interface_);
    registerInterface(&front_steer_jnt_pos_cmd_interface_);
  }

  void registerWheelInterface()
  {
    // actual wheel joints
    this->registerInterfaceHandles(
          jnt_state_interface_, rear_wheel_jnt_vel_cmd_interface_,
          rear_wheel_jnt_name_, rear_wheel_jnt_pos_, rear_wheel_jnt_vel_,
          rear_wheel_jnt_eff_, rear_wheel_jnt_vel_cmd_);
  }

  void registerSteerInterface()
  {
    // actual steer joints
    this->registerInterfaceHandles(
          jnt_state_interface_, front_steer_jnt_pos_cmd_interface_,
          front_steer_jnt_name_, front_steer_jnt_pos_, front_steer_jnt_vel_,
          front_steer_jnt_eff_, front_steer_jnt_pos_cmd_);

  }

  void registerInterfaceHandles(
          hardware_interface::JointStateInterface& _jnt_state_interface,
          hardware_interface::JointCommandInterface& _jnt_cmd_interface,
          const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff,  double& _jnt_cmd)
  {
    // register joint (both JointState and CommandJoint)
    this->registerJointStateInterfaceHandle(_jnt_state_interface, _jnt_name,
                                            _jnt_pos, _jnt_vel, _jnt_eff);
    this->registerCommandJointInterfaceHandle(_jnt_state_interface, _jnt_cmd_interface,
                                              _jnt_name, _jnt_cmd);
  }

  void registerJointStateInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      const std::string _jnt_name, double& _jnt_pos, double& _jnt_vel, double& _jnt_eff)
  {
    hardware_interface::JointStateHandle state_handle(_jnt_name,
                                                      &_jnt_pos,
                                                      &_jnt_vel,
                                                      &_jnt_eff);
    _jnt_state_interface.registerHandle(state_handle);

    ROS_WARN_STREAM("Registered joint '" << _jnt_name << " ' in the JointStateInterface");
  }

  void registerCommandJointInterfaceHandle(
      hardware_interface::JointStateInterface& _jnt_state_interface,
      hardware_interface::JointCommandInterface& _jnt_cmd_interface,
      const std::string _jnt_name, double& _jnt_cmd)
  {
    // joint command
    hardware_interface::JointHandle _handle(_jnt_state_interface.getHandle(_jnt_name),
                                            &_jnt_cmd);
    _jnt_cmd_interface.registerHandle(_handle);

    ROS_INFO_STREAM("Registered joint '" << _jnt_name << " ' in the CommandJointInterface");
  }

  float radians2degrees(float radians)
  {
    return (radians * (180 / PI));
  }

  float degrees2radians(float degrees)
  {
    return (degrees * PI) / 180;

  }

  double impose_speed_limit(double requested_speed, double max_forward_speed, double max_reverse_speed)
  {
    if (requested_speed > max_forward_speed)
    {
     requested_speed = max_forward_speed;
    }
    else if (requested_speed < max_reverse_speed * -1)
    {
     requested_speed = max_reverse_speed * -1;
    }

    return requested_speed;
  }

  double impose_angle_limit(double requested_angle_rad, double max_angle)
  {
    double requested_angle_deg = radians2degrees(requested_angle_rad);

    if (std::abs(requested_angle_deg) > max_angle)
    {
     requested_angle_deg = max_angle * (requested_angle_deg / std::abs(requested_angle_deg));
    }

    return degrees2radians(requested_angle_deg);
  }

  double speed2throttle(double speed, double max_forward_speed, double max_reverse_speed)
  {
    double throttle = 0;

    if (speed > 0)
    {
      throttle = speed / max_forward_speed;
    }
    else
    {
      throttle = speed / max_reverse_speed;
    }

    return throttle;
  }



private:
  // common
  hardware_interface::JointStateInterface jnt_state_interface_;// rear wheel
  //-- actual joint(single actuator)
  //---- joint name
  std::string rear_wheel_jnt_name_;
  //---- joint interface parameters
  double rear_wheel_jnt_pos_;
  double rear_wheel_jnt_vel_;
  double rear_wheel_jnt_eff_;
  //---- joint interface command
  double rear_wheel_jnt_vel_cmd_;
  //---- Hardware interface: joint
  hardware_interface::VelocityJointInterface rear_wheel_jnt_vel_cmd_interface_;
  //hardware_interface::JointStateInterface wheel_jnt_state_interface_;
  //


  // front steer
  //-- actual joint(single actuator)
  //---- joint name
  std::string front_steer_jnt_name_;
  //---- joint interface parameters
  double front_steer_jnt_pos_;
  double front_steer_jnt_vel_;
  double front_steer_jnt_eff_;
  //---- joint interface command
  double front_steer_jnt_pos_cmd_;
  //---- Hardware interface: joint
  hardware_interface::PositionJointInterface front_steer_jnt_pos_cmd_interface_;

  double max_steering_angle_;
  double max_wheel_speed_; // Wheel turns per second at full throttle
  double max_reverse_wheel_speed_; // Wheel turns per second at full reverse throttle
  double wheel_radius_;
  double max_linear_velocity_;

  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
