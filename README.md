
# kangaroo_x2_driver
ROS driver for Kangaroo X2 motor driver board

Takes a clean copy of the Kangaroo Arduino library provided by Dimension Engineering, and provides a reimplementation of Arduino.h in order to connect to the X2 through a linux serial port instead of Arduino serial. 

The intention is to use this to control the X2 from a Raspberry Pi through an FTDI USB-serial driver.

Update: code to use with Teensy for compatability with RC controller 


## Author
Tim Fan

original source code https://github.com/tim-fan/kangaroo_x2_driver

Edits by Valerie Pober

- added params for serial port connection and baud rate to launch file
- added joints and channels for rear axle
- updated command directions for wiring
- updated urdf file for 4-wheel implementation
- updated yaml file for 4-wheel implementation
- Teensy code for compatability with RC controller



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

## URDF File dimensions
In order to obtain best accuracy the following dimensions should be modified based on robot implementation:
- wheel_r
- length
- axle width

The following diagram indicates the dimensions refered to in the URDF file:

![URDF dimensions](https://github.com/vpober/ARL-SubtRobot/blob/master/urdf/Chassis_URDFdimensions.PNG)

## Teensy Setup

Teensyduino download for Arduino IDE compatability
https://www.pjrc.com/teensy/td_download.html

#### Add following libraries to Arduino Library directory

Kangaroo X2 Arduino library:
https://www.dimensionengineering.com/info/arduino

Timer library:
https://github.com/JChristensen/Timer/archive/v2.1.zip

#### File selection

*For use with RC controller:*
- If kangaroo is tuned in independent mode use “IndependentMode_RC-Serial_switch”
- If kangaroo is tuned in mixed mode use “MixedMode_RC-Serial_swtich”

To calibrate Max and Min RC constant values (D_MAX_RC, D_MIN_RC, T_MAX_RC, T_MIN_RC) use “PulseIn_RC_Calibrate” to determine values

*For serial only*

“TeensySerial”



