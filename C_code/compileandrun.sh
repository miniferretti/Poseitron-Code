sudo cd /home/pi/RobotCode/C_code

g++ -std=c++0x -pthread main.cc -lwiringPi IO/COM/SPI/SPI.cc IO/COM/CAN/CAN.cc IO/COM/SPI/Specific/SPI_CAN.cc IO/COM/SPI/Specific/SPI_DE0.cc IO/COM/TCS3472_I2C/TCS3472_I2C.cc -I . -o myprogram

./myprogram