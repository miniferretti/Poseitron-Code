cd ~/Desktop/WhiteSpirit_C

g++ -std=c++0x -pthread main.cc -lwiringPi IO/COM/SPI/SPI.cc IO/COM/CAN/CAN.cc IO/COM/SPI/Specific/SPI_CAN.cc IO/COM/SPI/Specific/SPI_DE0.cc -I . -o myprogram

./myprogram