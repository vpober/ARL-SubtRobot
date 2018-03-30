# kangaroo_x2_driver
ROS driver for Kangaroo X2 motor driver board

Takes a clean copy of the Kangaroo Arduino library provided by Dimension Engineering, and provides a reimplementation of Arduino.h in order to connect to the X2 through a linux serial port instead of Arduino serial. 

The intention is to use this to control the X2 from a Raspberry Pi through an FTDI USB-serial driver.

Kangaroo X2 Arduino library:
https://www.dimensionengineering.com/info/arduino

To test the connection to the motor driver:
```
rosrun kangaroo_x2_driver SpeedControlExample
```
(this will run motor 1 back and forward)

To create separate controllers for motors 1 and 2:
```
roslaunch independent_speed_control.launch
```
The controllers listen for speed commands of type std_msgs/Float64 on topics: 
/motor_1_controller/command
/motor_2_controller/command

