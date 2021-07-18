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
#include <sstream>

enum INDEX_WHEEL {
    INDEX_RIGHT = 0,
    INDEX_LEFT = 1
};

class Steerbot : public hardware_interface::RobotHW
{
public:
  Steerbot()
  : running_(true)
  , start_srv_(nh_.advertiseService("start", &Steerbot::startCallback, this))
  , stop_srv_(nh_.advertiseService("stop", &Steerbot::stopCallback, this))
  , ns_("/car/ackermann_steering_controller/")
  {

    // Intialize raw data
    this->cleanUp();
    this->getJointNames(nh_);
    this->registerHardwareInterfaces();

    nh_.getParam(ns_ + "wheel_separation_w", wheel_separation_w_);
    nh_.getParam(ns_ + "wheel_separation_h", wheel_separation_h_);
    ROS_INFO_STREAM("wheel_separation_w in test steerbot= " << wheel_separation_w_);
    ROS_INFO_STREAM("wheel_separation_h in test steerbot= " << wheel_separation_h_);

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
    std::ostringstream os;
    // directly get from controller
    os << rear_wheel_jnt_vel_cmd_ << ", ";
    os << front_steer_jnt_pos_cmd_ << ", ";

    //-- ackerman link
    const double h = wheel_separation_h_;
    const double w = wheel_separation_w_;

    if (rear_wheel_jnt_vel_cmd_ != 0.0 || front_steer_jnt_pos_cmd_ != 0.0)
      ROS_INFO_STREAM("Commands for joints: " << os.str());

  }

  void write()
  {
    std::ostringstream os;
    if (running_)
    {
      // wheels
      rear_wheel_jnt_pos_ += rear_wheel_jnt_vel_*getPeriod().toSec();
      rear_wheel_jnt_vel_ = rear_wheel_jnt_vel_cmd_;

      // steers
      front_steer_jnt_pos_ = front_steer_jnt_pos_cmd_;

      ROS_WARN_STREAM("Vel" << rear_wheel_jnt_vel_);
      ROS_WARN_STREAM("Pos" <<rear_wheel_jnt_pos_);

      // directly get from controller
      os << rear_wheel_jnt_vel_cmd_ << ", ";
      os << front_steer_jnt_pos_cmd_ << ", ";

    }
    else
    {
      // wheels
      rear_wheel_jnt_pos_= std::numeric_limits<double>::quiet_NaN();
      rear_wheel_jnt_vel_= std::numeric_limits<double>::quiet_NaN();

      // steers
      front_steer_jnt_pos_= std::numeric_limits<double>::quiet_NaN();
      front_steer_jnt_vel_= std::numeric_limits<double>::quiet_NaN();


      // wheels
      os << rear_wheel_jnt_pos_ << ", ";
      os << rear_wheel_jnt_vel_ << ", ";


      // steers
      os << front_steer_jnt_pos_ << ", ";
      os << front_steer_jnt_vel_ << ", ";
    }
    ROS_INFO_STREAM("running_ = " << running_ << ". commands are " << os.str());
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
    _nh.getParam(ns_ + "rear_wheel", rear_wheel_jnt_name_);
    ROS_WARN_STREAM("REAR wheel" << rear_wheel_jnt_name_);

  }

  void getSteerJointNames(ros::NodeHandle &_nh)
  {
    // steer joint to get angular command
    _nh.getParam(ns_ + "front_steer", front_steer_jnt_name_);

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
          rear_wheel_jnt_name_, rear_wheel_jnt_pos_, rear_wheel_jnt_vel_, rear_wheel_jnt_eff_, rear_wheel_jnt_vel_cmd_);
  }

  void registerSteerInterface()
  {
    // actual steer joints
    this->registerInterfaceHandles(
          jnt_state_interface_, front_steer_jnt_pos_cmd_interface_,
          front_steer_jnt_name_, front_steer_jnt_pos_, front_steer_jnt_vel_, front_steer_jnt_eff_, front_steer_jnt_pos_cmd_);

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




  // Wheel separation, wrt the midpoint of the wheel width:
  double wheel_separation_w_;
  // Wheel separation, wrt the midpoint of the wheel width:
  double wheel_separation_h_;

  std::string ns_;
  bool running_;

  ros::NodeHandle nh_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
};
