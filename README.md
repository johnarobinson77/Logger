# Logger

Utility to display and save the terminal output from microcontrollers like the Pi Pico

This repository holds the C++ code for a utility that opens a TTY port started by a microcontroller like the Pi Pico.  Its main advantages over using existing utilities like minicom are:
1. Logger can be started before the port has been created and it will open the port as soon as it is available.  If the microcontroller reset, it will realize that and wait for the port to be created again. This includes restarts for downloading new code.
2. It can add a timestamp to each line of output.  This is useful if the microcontroller is sending sensor data to be saved and the time in wad received is important.
3. It can send output to a file as well as displaying it.

Here is the current command line usage.
Usage:
Logger [-ts -nts] [-h] [-f <filename>] [-p <port>]
    -ts  add a time stamp to every lint printed (default)
    -nts do not add a timestamp
    -f <filename> send output to 'filename' 
    -p <port> open specified port
    -h   print this message
Note: if no port is specified, the default port name is used
      on Linux the default name is /dev/ttyACM0
      on the mac, the default name is dev/cu.usbmodem0000000000001

Compile with g++ Logger.cpp -o Logger.
  
This code has been run on Ubuntu (arm), Raspbian, and Mac OS but only with a Pi Pico microcontroller.
