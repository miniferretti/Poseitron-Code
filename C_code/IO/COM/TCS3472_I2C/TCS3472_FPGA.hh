#ifndef TCS3472_FPGA_HH
#define TCS3472_FPGA_HH

#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#define RED_REG 3
#define GREEN_REG 4
#define BLUE_REG 5
#define CLEAR_REG 6
#define SENSOR_NUM 2

void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(float *r, float *g, float *b);
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);
void sensorSelect(int bus);
uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
float powf(const float x, const float y);
void colorSensorReset();

#endif