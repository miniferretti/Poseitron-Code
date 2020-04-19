/*************************************************************************
* Header file for MyDynamixel                                            *
* Version 1.00                                                           *
**************************************************************************/
#ifndef MYDYNAMIXEL_H_
#define MYDYNAMIXEL_H_
/*************************************************************************
* Library declarations				                                     *
**************************************************************************/
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
/*************************************************************************
* Types Definitions					                                     *
**************************************************************************/
typedef unsigned char Byte; //integer of 8 bits
/*************************************************************************
* Hardware Module Registers			                                     *
**************************************************************************/
#define COMMUNICATION_STATUS_REG 0x00
#define START_COMMUNICATION_REG 0x04
#define TXD_BYTES_REG 0x05
#define TXD_PARAMETERS_REG 0x06
/*************************************************************************
* Dynamixel Registers				                                     *
**************************************************************************/
#define CW_ANGLE_LIMIT_REG 0x06
#define CCW_ANGLE_LIMIT_REG 0x08
#define LED_REG 0x19
#define CW_COMPLIANCE_MARGIN_REG 0x1A
#define CCW_COMPLIANCE_MARGIN_REG 0x1B
#define CW_COMPLIANCE_SLOPE_REG 0x1C
#define CCW_COMPLIANCE_SLOPE_REG 0x1D
#define GOAL_POSITION_REG 0x1E
#define MOVING_SPEED_REG 0x20
#define TORQUE_LIMIT_REG 0x22
#define PRESENT_POSITION_REG 0x24
#define PRESENT_SPEED_REG 0x26
#define PRESENT_LOAD_REG 0x28
#define LOCK_REG 0x2F

/************************************************************************
 * SPI registers
*************************************************************************/
#define DATA_REG 0x89
#define ADDRESS_REG 0x8A
#define READ_REG 0x0B
#define WRITE_BIT 0b00000001
#define READ_BIT  0b00000010
#define TRANSITION_BIT 0b00000000

/*************************************************************************
* Functions Prototypes				                                     *
**************************************************************************/
int Send_Instruction_Packet(Byte ID, Byte Length, Byte Instruction, Byte P1, Byte P2, Byte P3);
int Get_Status_Packet(Byte *ID, Byte *Length, Byte *Error, Byte *P1, Byte *P2, Byte *Checksum, Byte *Fail);

Byte Set_Parameter(Byte ID, Byte Length, Byte Address, int Parameter);
int Get_Parameters(Byte ID, Byte Address, Byte N);

int read_data(unsigned char reg);
void reset_dynamixel();
void reset_buffer(unsigned char *buffer);

#endif /*MYDYNAMIXEL_H_*/
