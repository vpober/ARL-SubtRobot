# kangaroo_x2_driver
ROS driver for Kangaroo X2 motor driver board

Takes a clean copy of the Kangaroo Arduino library provided by Dimension Engineering, and provides a reimplementation of Arduino.h in order to connect to the X2 through a linux serial port instead of Arduino serial. 

The intention is to use this to control the X2 from a Raspberry Pi through an FTDI USB-serial driver.

Kangaroo X2 Arduino library:
https://www.dimensionengineering.com/info/arduino
