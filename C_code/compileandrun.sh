sudo cd /home/pi/RobotCode/C_code

sudo g++ -std=c++0x -pthread main.cc -lwiringPi IO/COM/SPI/SPI.cc IO/COM/SPI/Specific/SPI_CAN.cc IO/COM/SPI/Specific/SPI_DE0.cc IO/COM/TCS3472_I2C/TCS3472_I2C.cc IO/COM/CAN/CAN_Alternate.cc IO/Speed_Controller/CtrlStruct.cc IO/Speed_Controller/SpeedController.cc IO/Odometry/Odometry.cc IO/Sens_Array/Sens_Array.cc IO/Color_Array/Color_Array.cc IO/Calibration/Calibration.cc IO/Mid_level_controller/Avoid150.cc -I . -o myprogram

sudo ./myprogram