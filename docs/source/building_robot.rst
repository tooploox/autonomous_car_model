******************
Building the robot
******************
The core of the project is an actual robot. Here's a very short guide related to building it yourself.

Essential hardware
==================
This is a minimal list just to get started. You can extend it according to your needs. You can read more detailed description of parts in our `blogpost on medium <https://medium.com/@adam.slucki/building-an-autonomous-car-76d8b9dfb86b>`_

1. **Jetson Nano**. Currently we're using 2GB version.
2. **Chassis**. It can be anything that will allow you to build a robot that resembles a car. We're using Tamiya TT02 chassis.
3. **Motor**. We recommend a motor with encoder because it'll be useful for odometry. Please get familiar with motor types and specs before buying.
4. **Servomechanism**
5. **Electronic speed controller (ESC)**. This should match your motor. If you can find a controller with BEC rated for 3A you could use it to power Jetson from the same battery as the motor and servo. But it may cause some issues.
6. **Battery** LiPo batteries are the most popular choice. Make sure that the batter matches your ESC, motor and fits into your chassis. You'll also need a charger for the battery.
7. **PowerBank**. We're using a powerbank to as to power our Jetson board.
8. Wires, connectors, etc.

.. note::

    Estimated cost of those essential components is: **$450**

    You can also build the robot based on Nvidia jetracer project: https://github.com/NVIDIA-AI-IOT/jetracer/blob/master/docs/tamiya/bill_of_materials.md
    But it might be difficult to install additional sensors with that design as there is almost no room left on the deck. You also may not need the radio controller that is quite costly.

Assembly
========
The project doesn't assume any specific design beyond the robot being a model of a car so we cannot provide the exact instruction. But the general steps are:

1. **Assemble the chassis.** If you bought Tamiya chassis build it according to the instruction provided by Tamiya.
2. **Build a deck**. You can 3D print our model: https://www.thingiverse.com/thing:4808250 You'll need 2 copies of it that you can glue together. The deck can be mounted to chassis in the same way as a body shell (no modifications to Tamiya TT02 chassis are required).
3. **Mount the jetson and any sensors** you plan to use on the deck using a glue or other solutions.
4. **Connect wires:**

    * Connect power and ground wire of BEC of your electronic speed controller to the servomechanism.
    * Connect ground of BEC to Jetson ground.
    * Connect servomechanism PWM wire to Jetson pin 32 or 33 (we used 32).
    * Connect BEC PWM wire to Jetson pin 32 or 33 (we used 33).

    .. note::
        You can read more about PWM signal and enabling PWM on Jetson in our `medium post <https://medium.com/@adam.slucki/building-an-autonomous-car-3321b2be101e>`_

Manual steering
===============
You can control the robot using web GUI. To do so:

1. Start the required nodes:

    .. code-block:: bash

        $ roslaunch car_bringup start_all.launch

    .. warning::
        You should check the parameters specified in car_bringup/launch/start_all.launch

2. Turn on your robot (connect the battery and turn on the ESC)

3. Open robot_gui_bridge/gui/gui.html in your browser.

    .. warning::
       You'll have to manually change IP address of your Jetson in the html code.

4. That's it, you're ready to play with your robot!

    .. figure:: images/robot_gui.gif
        :scale: 120 %

        Web GUI for manual steering and viewing sensors data

