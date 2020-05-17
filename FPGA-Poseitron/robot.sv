
//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================

module robot(

	//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	PI,
	PI_IN,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	BRIDGE,
	BRIDGE_IN 
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		PI;
input 		     [1:0]		PI_IN;

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout 		    [33:0]		BRIDGE;
input 		     [1:0]		BRIDGE_IN;

////////////////////////////////////////////////////////////////
//
//   Written by: Matteo Ferretti di CastelFerretto
//
//   Credits to: Louis Devillez for the quadartics counters
//
////////////////////////////////////////////////////////////////

      // F - front | R - rear
      // L - left  | R - Right
      logic quadA_FL, quadB_FL, quadA_RL, quadB_RL, quadA_odoR, quadB_odoR, quadA_odoL, quadB_odoL;
		  
		  // logic reset = 1'b1;
		  logic i2c_ack_error;
		  logic [15:0] clear,red,green,blue;
      logic [31:0] ports_control;
		  
        assign quadA_FL = BRIDGE_IN[0];
        assign quadB_FL = BRIDGE_IN[1];

        assign quadA_RL = BRIDGE[2];
        assign quadB_RL = BRIDGE[4];

        // roue odometrique droite
        assign quadA_odoR = BRIDGE[9];
        assign quadB_odoR = BRIDGE[11];

        // roue odometrique gauche
        assign quadA_odoL = BRIDGE[13];
        assign quadB_odoL = BRIDGE[15];

        assign LED[7:0] =ports_control[7:0];//data_out[7:0];//{quadA_FL,quadB_FL,quadA_RL,quadB_RL,quadA_odoL,quadB_odoL,quadA_odoR,quadB_odoR};

        // Clocks


        //---------------------------Quadratic Counters-----------------------//
        logic PLL_CLOCK;
        my_pll pll_clock(CLOCK_50, PLL_CLOCK);


        logic [31:0] count_FL, count_RL, count_odoR, count_odoL;
        quad encoder_FL(CLOCK_50, 1'b0, quadA_FL, quadB_FL, count_FL);
        quad encoder_RL(CLOCK_50, 1'b0, quadA_RL, quadB_RL, count_RL);
        quad odometerR(CLOCK_50,1'b0,quadA_odoR,quadB_odoR,count_odoR);
        quad odometerL(CLOCK_50,1'b0,quadA_odoL,quadB_odoL,count_odoL);
    




        logic [31:0] speed_FL, speed_RL, speed_FR, speed_RR;
      

        speed speed_counter_FL(PLL_CLOCK, count_FL, speed_FL);
        speed speed_counter_RL(PLL_CLOCK, count_RL, speed_RL);
      

        
        //-----------------------------SPI Controller---------------------//
        logic        spi_clk, spi_cs, spi_mosi, spi_miso;
        logic [31:0] data_write, data_read;
        logic [3:0]  data_addr;
        logic [31:0] dyna_read;
        logic [31:0] reg_addr;
        logic [31:0] dyna_write;
        logic [31:0] data_out;

        spi_slave_mu spi_slave_instance(CLOCK_50, 
                                        spi_clk,        //Clock signal comming from the Raspberry pi 
                                        spi_cs,         //Chip select signal from the Raspberry Pi
                                        spi_mosi,       //Master output, Slave input signal from the Raspberry Pi
                                        spi_miso,       //Master input, Slave output signal from the Raspberry Pi
                                        // DATA TO SEND TO RPi :
                                        speed_FL,       //Left speed value input register
                                        speed_RL,       //Right speed value input register
                                        count_odoR,     //Right odometric counter input register
                                        count_odoL,     //Left odometric counter input register
                                        red,            //Red color amplitude input register
                                        green,          //Green color amplitude input register
                                        blue,           //Blue color amplitude input register 
                                        clear,          //Luminance amplitude input register
                                        dyna_read,      //Dynamixel status packet input register
                                        // DATA TO RECIEVE FROM RPi :
                                        data_out,       //Color sensor selection output register
                                        ports_control,  //Ports control output register of the MCP23017
                                        dyna_write,     //Command packet output register for the Dynamixel
                                        reg_addr        //Action output register for the UART controller module
                                        );

        assign spi_clk                  = PI[11];   // SCLK = pin 16 = RPi_11
        assign spi_cs                   = PI[9];    // CE0  = pin 14 = RPi_9
        assign spi_mosi                 = PI[15];   // MOSI = pin 20 = RPi_15
        assign PI[13]                   = spi_miso;     // MISO = pin 18 = RPi_13 

		  logic spi_clk1, spi_cs1, spi_mosi1, spi_miso1;
     // assign spi_clk1 = PI_IN[0];
      assign spi_miso1 = PI[1];
     // assign spi_mosi1 = PI_IN[1];
      assign spi_cs1 = PI[2];

      wire direction_port;
		  
		  
		assign BRIDGE[14] = direction_port;
		  
		  
		  
		  
	 // BRIDGE[31] = SDA; //SDA
	 // BRIDGE[33] = SCL; //SCL
		 
  
	  //------------------I2C Controller----------------------------//	 
		pmod_color_sensor daColorSensor(.clk(CLOCK_50), 
                                    .reset_n(PI[7]),                            //Reset pin of the Raspberry Pi
                                    .scl(BRIDGE[33]),                           //Real scl line of the i2c line 
                                    .sda(BRIDGE[31]),                           //Real sda line of the i2c line
                                    .i2c_ack_err(i2c_ack_error),                //Bit used to notify any transmission error
                                    .clear(clear),                              //Luminance register of the selected color sensor 
                                    .red(red),                                  //Red amplitude of the selected color sensor
                                    .green(green),                              //Green amplitude of the selected color sensor 
                                    .blue(blue),                                //Blue amplitude of the selected color sensor 
                                    .sensor_select(data_out[7:0]),              //8 bit register used to select the corresponding color sensor
                                    .portA_output_select(ports_control[7:0]),   //8 bit register to control the A section of the outputs of the MCP23017
                                    .portB_output_select(ports_control[15:8])   //8 bit register to control the B section of the outputs of the MCP23017
                                    );
		 


  //--------------------Dynamixel Controller-------------------------//
  UART_Dynamixel myDyna(CLOCK_50,
                        PI[5],          //à définir... //Reset pin from the Pi, this pin corresponds to pin 22 of the Raspberry Pi
                        reg_addr[8],    //Bit used to tel the Uart controller that we write to the Dynamixel
                        reg_addr[9],    //Bit used to tel the Uart controller that we read from the Dynamixel
                        reg_addr[2:0],  //Address bits to specify the register of the Uart controller
                        dyna_write,     //Commands to send to the Dynamixel from the SPI
                        dyna_read,      //Command from the Dynamixel to send to the SPI
                        BRIDGE[10],     //Real outputs of the FPGA to the Dynamixel RX
                        BRIDGE[12],     //Real outputs of the FPGA to the Dynamixel TX
                        direction_port,      //Real outputs of the FPGA to the Dynamixel CTRL
                        ID_flag
                        );

  logic [7:0] ID_flag;

endmodule
