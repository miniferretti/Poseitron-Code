#include "IO/COM/TCS3472_I2C/TCS3472_FPGA.hh"

void getRawData(uint16_t *r, uint16_t *g, uint16_t *b,
                uint16_t *c)
{

    unsigned char buffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    buffer[0] = 0x06;

    wiringPiSPIDataRW(0, buffer, 5);

    *c = (uint16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4]);

    buffer[0] = 0x03;

    wiringPiSPIDataRW(0, buffer, 5);

    *r = (uint16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4]);

    buffer[0] = 0x04;

    wiringPiSPIDataRW(0, buffer, 5);

    *g = (uint16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4]);

    buffer[0] = 0x05;

    wiringPiSPIDataRW(0, buffer, 5);

    *b = (uint16_t)((uint16_t)buffer[3] << 8 | (uint16_t)buffer[4]);
}

void colorSensorReset()
{
    pinMode(21, OUTPUT);
    digitalWrite(21, 1);
    digitalWrite(21, 0);
    delay(1);
    digitalWrite(21, 1);
}

void getRGB(float *r, float *g, float *b)
{
    uint16_t red, green, blue, clear;
    getRawData(&red, &green, &blue, &clear);
    uint32_t sum = clear;

    // Avoid divide by zero errors ... if clear = 0 return black
    if (clear == 0)
    {
        *r = *g = *b = 0;
        return;
    }

    *r = (float)red / sum * 255.0;
    *g = (float)green / sum * 255.0;
    *b = (float)blue / sum * 255.0;
}

uint16_t calculateColorTemperature(uint16_t r, uint16_t g,
                                   uint16_t b)
{
    float X, Y, Z; /* RGB to XYZ correlation      */
    float xc, yc;  /* Chromaticity co-ordinates   */
    float n;       /* McCamy's formula            */
    float cct;

    if (r == 0 && g == 0 && b == 0)
    {
        return 0;
    }

    /* 1. Map RGB values to their XYZ counterparts.    */
    /* Based on 6500K fluorescent, 3000K fluorescent   */
    /* and 60W incandescent values for a wide range.   */
    /* Note: Y = Illuminance or lux                    */
    X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
    Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
    Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

    /* 2. Calculate the chromaticity co-ordinates      */
    xc = (X) / (X + Y + Z);
    yc = (Y) / (X + Y + Z);

    /* 3. Use McCamy's formula to determine the CCT    */
    n = (xc - 0.3320F) / (0.1858F - yc);

    /* Calculate the final CCT */
    cct =
        (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

    /* Return the results in degrees Kelvin */
    return (uint16_t)cct;
}

float powf(const float x, const float y)
{
    return (float)(pow((double)x, (double)y));
}

uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b)
{
    float illuminance;

    /* This only uses RGB ... how can we integrate clear or calculate lux */
    /* based exclusively on clear since this might be more reliable?      */
    illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

    return (uint16_t)illuminance;
}

void sensorSelect(int bus)
{
    unsigned char buffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    buffer[0] = 0x87;
    buffer[4] = 1 << (uint8_t)bus;

    wiringPiSPIDataRW(0, buffer, 5);
}