# BLDC_Controller
Program written for STM32F401RE MCU on a Nucleo64 board. It was designed to drive EMAX CF2822 brushless DC motor with 8x4.7 propeller  without using any sensors. User can control the speed of motor and read motor's parameters via serial port with baudrate of 115200.

The Nucleo board is connected to the specially designed for this purpose PCB containing all the necessary hardware and connectors. 

Software has implementation of PI regulator in order to control the motor's speed.
