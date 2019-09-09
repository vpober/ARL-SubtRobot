
# kangaroo_x2_driver
ROS driver for Kangaroo X2 motor driver board

Takes a clean copy of the Kangaroo Arduino library provided by Dimension Engineering, and provides a reimplementation of Arduino.h in order to connect to the X2 through a linux serial port instead of Arduino serial. 

The intention is to use this to control the X2 from a Raspberry Pi through an FTDI USB-serial driver.

Kangaroo X2 Arduino library:
https://www.dimensionengineering.com/info/arduino

## Author
Tim Fan

original source code https://github.com/tim-fan/kangaroo_x2_driver

Edits by Valerie Pober

-added params for serial port connection and baud rate to launch file

-added joints and channels for rear axle

-updated command directions for wiring

-updated urdf file for 4-wheel implementation

-updated yaml file for 4-wheel implementation



## Test the connection to the motor driver:
```
rosrun kangaroo_x2_driver SpeedControlExample
```
(this will run motor 1 back and forward)

## Create separate controllers for motors 1, 2, 3, and 4:
```
roslaunch kangaroo_x2_driver independent_speed_control.launch
```
The controllers listen for speed commands of type std_msgs/Float64 on topics: 
```
/front_right_motor_joint/command
/front_left_motor_joint/command
/rear_right_motor_joint/command
/rear_left_motor_joint/command
```

## Differential drive control:
```
roslaunch kangaroo_x2_driver diff_drive_control.launch
```
The differential-drive controller will listen for twist velocity commands published to topic `/diff_drive_controller/cmd_vel`.


## Kangaroo Setup
Ensure the kangaroo board is configured for independent mode (dip-switch 4 is ON) and digital input (dip-switch 1 is ON). Also ensure the board has been tuned with these settings.

Change Kangaroo address for rear axle to 129 and Channel 1 Name to '3' and Channel 2 Name to '4' in Describe software.

Enable multi-kangaroo mode for both in Describe software and daisy-chain Kangaroos.
