sudo cd /home/pi/RobotCode/C_code

sudo g++ -std=c++0x -pthread main.cc -lwiringPi IO/COM/SPI/SPI.cc IO/COM/CAN/CAN.cc IO/COM/SPI/Specific/SPI_CAN.cc IO/COM/SPI/Specific/SPI_DE0.cc IO/COM/TCS3472_I2C/TCS3472_I2C.cc IO/COM/CAN/CAN_Alternate.cc IO/Speed_Controller/CtrlStruct.cc IO/Speed_Controller/SpeedController.cc IO/Odometry/Odometry.cc -I . -o myprogram

sudo ./myprogram