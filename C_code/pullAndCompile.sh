#!/bin/bash

git branch

git pull

echo "Compiling"

sudo g++ -std=c++0x -pthread -I/usr/include/python3.7m main.cc -lpython3.7m -lwiringPi IO/COM/SPI/SPI.cc IO/COM/SPI/Specific/SPI_CAN.cc IO/COM/SPI/Specific/SPI_DE0.cc IO/COM/TCS3472_I2C/TCS3472_FPGA.cc IO/COM/CAN/CAN_Alternate.cc IO/Speed_Controller/CtrlStruct.cc IO/Speed_Controller/SpeedController.cc IO/Odometry/Odometry.cc IO/Sens_Array/Sens_Array.cc IO/Color_Array/Color_Array.cc IO/Calibration/Calibration.cc IO/Mid_level_controller/Avoid150.cc IO/Dynamixel/DynamixelFunctions.cc IO/Dynamixel/MyDynamixel.cc IO/Pincher_Demo/Pincher_Demo.cc IO/Calibration/odo_calibration.cc -I . -o run

echo "Compilation done"