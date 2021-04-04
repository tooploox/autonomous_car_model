# Autonomous car model [WIP]
![project overview](media/overview.JPG)  
We're building a model of an autonomous car using ROS framework. The idea is to have a fully functional car in 1:10 scale that we can use for experiments, development and testing of Computer Vision and Machine Learning algorithms.

## Environment
Currently, we're working on ROS Melodic running on **Jetson Nano**. The distribution doesn't support Python 3, so we'll probably switch to Noetic at some point. There are no plans on using ROS 2 at the moment.

### Preparing the environment
1. Install ROS as described here: http://wiki.ros.org/melodic/Installation/Ubuntu
2. Install additional dependencies (in the future it should be automated):  
`apt-get install ros-melodic-rosbridge-server`
3. Activate the workspace by running `devel/setup.bash` script. For convenience, it's recommended to add this command to bashrc script.  
   `source /media/adam/data/Projects/car_robot/catkin_ws/devel/setup.bash`
4. Build the project:
Navigate to the [catkin_ws](catkin_ws) directory and run `catkin_make`
   
## Simulation
As using the actual robot is to always possible or convenient we recreated it in the simulated environment. This way it should be easier to develop and test solutions that can be then executed on the real car model as well.

### Model description
You can visualize the robot model in rozviz (tested in ROS Noetic) and interact with the model's joints using a simple GUI.
To do so run:  
`roslaunch robot_description urdf_visualize.launch`

You should see a visualization like this:
![project overview](media/urdf_viz.png)

### Gazebo simulation
Not implemented yet
   
## Using ROS on actual car model
1. (optional) Connect the sensors, motor and servo to pins as specified in the [launch file](catkin_ws/src/car_bringup/launch/start_all.launch) (or modify the file accordingly.) You can run the code on Jetson without any hardware connected to it.
2. From anywhere in the system run:  
`roslaunch car_bringup start_all.launch'
3. You can control the car with the [web GUI](catkin_ws/src/robot_gui_bridge/gui/gui.html). Simply modify the IP address in the sourcecode and open the html file in your browser. Note: of course you can open that file on any machine that can access your Jetson at its IP address.

## Current state/functionality
You'll be able to:
1. Manually steer your car using the [nipple.js](https://yoannmoi.net/nipplejs/) joystick
2. Access a camera stream (the settings are hardcoded for raspberry camera, using different camera may require some modifications).
3. Access ultrasonic sensors readings.
4. Access accelerometer and gyroscope readings.

## TODO list
We're just learning ROS so in its current state the organisation of the code is not ideal, and some solutions may suboptimal/have a temporary character.  
- [ ] Reorganise the packages, so the structure makes more sense
- [ ] Replace incorrectly used Twist messages published on \cmd_vel topic with the [ackermann_msg](http://wiki.ros.org/ackermann_msgs).
- [ ] Add robot description and support Gazebo simulation
- [ ] Mock GPIO imports, so the code can be executed on a PC.
- [ ] Refactor the communication with ultrasonic sensors, so they use interruptions instead of the `while loops`.
- [ ] Add support for additional sensors - magnetometer, motor encoder, possibly motor temperature sensor (analog <> digital converter required).
- [ ] Visualize the car's pitch, roll, yaw and heading in real time using OpenGL.
- [ ] Implement a basic autonomous logic (ex. brake when any of the ultrasonic sensors shows a small distance).

## Plans for the future
1. Install Lidar, run SLAM
2. Implement path planning
3. Implement image segmentation, drivable area detection
4. Design environment-specific autonomous logic.
5. ...