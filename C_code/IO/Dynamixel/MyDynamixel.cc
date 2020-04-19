/*************************************************************************
* ELME 2001 : Electromechanical Project                                  *
* Copyright (c) 2015 UCL							                     *
**************************************************************************
* MyDynamixel.c            											     *
* 	This file contains several functions used to communicate with the 	 *
* 	Dynamixels. They've been tested on AX-12 with a baudrate of 57600    *
* 	bps with a NIOS/e and a NIOS/f.	                                     *
**************************************************************************
* Written by Benjamin Delhaye, adapted by Matteo Ferretti						 					 *
* Version : 2.00 - 11 Aug 2015                                           *
**************************************************************************/
#include "MyDynamixel.h"
/*********************************************************************************************************
 *********************************************************************************************************
 *********************************************************************************************************/

/*
 * Description: Function used to send an Instruction Packet to a Dynamixel thanks to hardware modules.
 * 				The bytes are first write into registers. The START_COMMUNICATION_REG is then set to 1
 * 				to begin the communication. The first bit of COMMUNICATION_STATUS_REG is set to 0 if a
 * 				communication is in progress or to 1 when the communication is done.
 *
 * @Inputs: 	Bytes of an Instruction Packet (see Dynamixel datasheet). If you want to use less than 3
 * 				parameters, insert the correct Byte Length and set the other parameter(s) (P3 and/or P2)
 * 				to 0.
 *
 * @Output: 	return 1 if there's no communication in progress, after putting Bytes into correct
 * 				registers and starting the communication.
 * 				return -1 if the hardware modules are already busy with a communication. In this case,
 * 				you have to wait the communication to finish.
 */
int Send_Instruction_Packet(Byte ID, Byte Length, Byte Instruction, Byte P1, Byte P2, Byte P3)
{
	char communication_available = (char)(read_data(COMMUNICATION_STATUS_REG) & 0x00000001);
	unsigned char buffer[5] = {0, 0, 0, 0, 0};

	if (communication_available)
	{
		Byte Checksum = ~(ID + Length + Instruction + P1 + P2 + P3);

		printf("On envoie l'instru\r\n");

		//	IOWR(UART_DYNAMIXEL_BASE, START_COMMUNICATION_REG, 0); //Reset of the start flag

		buffer[0] = ADDRESS_REG; //dynamixel register
		buffer[3] = WRITE_BIT;	 //perfoming a write action to corresponding register
		buffer[4] = START_COMMUNICATION_REG;

		wiringPiSPIDataRW(0, buffer, 5);

		buffer[0] = DATA_REG; //data to write register
		buffer[1] = 0;
		buffer[2] = 0;
		buffer[3] = 0;
		buffer[4] = 0;

		wiringPiSPIDataRW(0, buffer, 5);

		//	IOWR(UART_DYNAMIXEL_BASE, TXD_BYTES_REG, (Checksum << 24 | Instruction << 16 | Length << 8 | ID)); //Write instructions into a register

		buffer[0] = ADDRESS_REG; //dynamixel register
		buffer[3] = WRITE_BIT;	 //perfoming a write action to corresponding register
		buffer[4] = TXD_BYTES_REG;

		wiringPiSPIDataRW(0, buffer, 5);

		buffer[0] = DATA_REG;
		buffer[1] = Checksum;
		buffer[2] = Instruction;
		buffer[3] = Length;
		buffer[4] = ID;

		printf("ID: 0x%x ; Length: 0x%x ; Instruction: 0x%x ; Checksum: 0x%x \n", buffer[4], buffer[3], buffer[2], buffer[1]);

		wiringPiSPIDataRW(0, buffer, 5);

		buffer[0] = ADDRESS_REG; //dynamixel register
		buffer[3] = WRITE_BIT;	 //perfoming a write action to corresponding register
		buffer[4] = TXD_PARAMETERS_REG;

		wiringPiSPIDataRW(0, buffer, 5);

		buffer[0] = DATA_REG; //data to write register
		buffer[1] = 0;
		buffer[2] = P3;
		buffer[3] = P2;
		buffer[4] = P1;

		wiringPiSPIDataRW(0, buffer, 5);

		usleep(10); // Wait for the hardware to get the data
		//	IOWR(UART_DYNAMIXEL_BASE, START_COMMUNICATION_REG, 1); //Set start flag

		buffer[0] = ADDRESS_REG; //dynamixel register
		buffer[3] = WRITE_BIT;	 //perfoming a write action to corresponding register
		buffer[4] = START_COMMUNICATION_REG;

		wiringPiSPIDataRW(0, buffer, 5);

		buffer[0] = DATA_REG; //data to write register
		buffer[1] = 0;
		buffer[2] = 0;
		buffer[3] = 0;
		buffer[4] = 0x01;

		wiringPiSPIDataRW(0, buffer, 5);

		usleep(10); // Wait for the hardware to begin

		return 1;
	}
	else
		return -1;
}
/*
 * Description: Function used to get a Status Packet send by a Dynamixel. After sending an Instruction
 * 				Packet, the hardware modules automatically save the Status Packet into registers. When
 * 				the Bytes are ready in the registers, the first bit of COMMUNICATION_STATUS_REG is set to
 * 				1. During the communication, the first bit of COMMUNICATION_STATUS_REG is set to 0. If the
 * 				requested Status Packet	only contains 2 parameters, P2 or/and P1 is/are set to 0x00.
 * 				If the Dynamixel doesn't answer to the Instruction Packet, it means that it hasn't received
 * 				this one. In this case, the second bit of COMMUNICATION_STATUS_REG is set to 1 and written in
 * 				the pointer Byte* Fail.
 *
 * @Inputs:	 	Pointer of Bytes of a Status Packet (see Dynamixel datasheet) and Byte* Fail.
 *
 * @Output: 	return 1 if the Bytes are available in the registers after filling the pointers with the Bytes
 * 				send by the Dynamixel. The second bit of COMMUNICATION_STATUS_REG is written into the pointer
 * 				"Fail" to know if the Dynamixel has received the packet and has answered.
 * 				return -1 if the Bytes aren't ready in the registers.
 */
int Get_Status_Packet(Byte *ID, Byte *Length, Byte *Error, Byte *P1, Byte *P2, Byte *Checksum, Byte *Fail)
{
	char data_available = (char)(read_data(COMMUNICATION_STATUS_REG) & 0x00000001);

	if (data_available)
	{
		printf("Les donnéés sont dispo\r\n");
		*ID = (Byte)(read_data(1) & 0x000000FF);
		*Length = (Byte)((read_data(1) & 0x0000FF00) >> 8);
		*Error = (Byte)((read_data(1) & 0x00FF0000) >> 16);
		*Checksum = (Byte)((read_data(1) & 0xFF000000) >> 24);
		*P1 = (Byte)((read_data(2) & 0x000000FF));
		*P2 = (Byte)((read_data(2) & 0x0000FF00) >> 8);
		*Fail = (Byte)((read_data(COMMUNICATION_STATUS_REG) & 0x00000002) >> 1);
		return 1; // If received, return 1 and write bytes into the arguments
	}
	else // If not received, return -1
		return -1;
}
/*********************************************************************************************************
 *********************************************************************************************************
 *********************************************************************************************************/

/*
 * Description: Example of how to use "Send_Instruction_Packet" and "Get_Status_Packet" to write parameters
 * 				into registers in a Dynamixel. Feel free to modify it or improve it.
 * 				Please read the Dynamixel datasheet for more informations.
 */
Byte Set_Parameter(Byte ID, Byte Length, Byte Address, int Parameter)
{
	Byte ID_read, Length_read, Error, P1, P2, Checksum, Fail;
	//	printf("ID: 0x%x ; Length: 0x%x ; Address: 0x%x ; Parameter: 0x%x \n", ID, Length, Address, (Byte)Parameter);

	while (Send_Instruction_Packet(ID, Length, 0x03, Address, Parameter, Parameter >> 8) == -1)
		;
	while (Get_Status_Packet(&ID_read, &Length_read, &Error, &P1, &P2, &Checksum, &Fail) == -1)
		;

	printf("ID: 0x%x ; Length: 0x%x ; Error: 0x%x ; P1: 0x%x ; P2: 0x%x ; Checksum: 0x%x ; Fail: 0x%x \n", ID_read, Length_read, Error, P1, P2, Checksum, Fail);

	return Fail;
}

/*
 * Description: Example of how to use "Send_Instruction_Packet" and "Get_Status_Packet" to read parameters
 * 				into registers in a Dynamixel. Feel free to modify it or improve it.
 * 				Please read the Dynamixel datasheet for more informations.
 */
int Get_Parameters(Byte ID, Byte Address, Byte N)
{
	Byte ID_read, Length_read, Error, P1, P2, Checksum_read, Fail;

	while (Send_Instruction_Packet(ID, 0x04, 0x02, Address, N, 0x00) == -1)
		;
	while (Get_Status_Packet(&ID_read, &Length_read, &Error, &P1, &P2, &Checksum_read, &Fail) == -1)
		;

	//printf("ID: 0x%x ; Length: 0x%x ; Error: 0x%x ; P1: 0x%x ; P2: 0x%x ; Checksum: 0x%x ; Fail: 0x%x \n",ID_read, Length_read, Error, P1, P2, Checksum_read, Fail);

	return (P2 << 8) + P1;
}

int read_data(unsigned char reg)
{

	int communication_available; // = (IORD(UART_DYNAMIXEL_BASE, COMMUNICATION_STATUS_REG) & 0x00000001);

	unsigned char buffer[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

	buffer[0] = ADDRESS_REG; //register containing the adresses of the dynamixel
	buffer[3] = READ_BIT;	 //bit 0 is the write bit, bit 1 is the read bit. Here, we perform a reading
	buffer[4] = reg;

	send_reset_buffer(buffer);

	buffer[0] = READ_REG; //read register for the retrieving of the status of the dynamixel

	send_reset_buffer(buffer);

	communication_available = (int)((uint32_t)buffer[1] << 24 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]);

	return communication_available;
}

void reset_dynamixel()
{
	pinMode(22, OUTPUT);
	digitalWrite(22, 0);
	delay(100);
	digitalWrite(22, 1);
	delay(100);
	digitalWrite(22, 0);
}

void send_reset_buffer(unsigned char *buffer)
{
	unsigned char add = buffer[0];

	wiringPiSPIDataRW(0, buffer, 5);

	if ((add & 0b10000000) >> 7)
	{

		for (int i = 0; i < 5; i++)
		{
			buffer[i] = 0;
		}
	}
}