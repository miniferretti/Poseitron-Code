
//=======================================================
//  MyMiniBot
//=======================================================

module MyDE0_Nano(

//////////// CLOCK //////////
input logic 		          		CLOCK_50,

//////////// LED //////////
output logic		     [7:0]		LED,

//////////// KEY //////////
input logic 		     [1:0]		KEY,

//////////// SW //////////
input logic 		     [3:0]		SW,

//////////// SDRAM //////////
output logic		    [12:0]		DRAM_ADDR,
output logic		     [1:0]		DRAM_BA,
output logic		          		DRAM_CAS_N,
output logic		          		DRAM_CKE,
output logic		          		DRAM_CLK,
output logic		          		DRAM_CS_N,
inout logic 		    [15:0]		DRAM_DQ,
output logic		     [1:0]		DRAM_DQM,
output logic		          		DRAM_RAS_N,
output logic		          		DRAM_WE_N,

//////////// EPCS //////////
output logic		          		EPCS_ASDO,
input logic 		          		EPCS_DATA0,
output logic		          		EPCS_DCLK,
output logic		          		EPCS_NCSO,

//////////// Accelerometer and EEPROM //////////
output logic		          		G_SENSOR_CS_N,
input logic 		          		G_SENSOR_INT,
output logic		          		I2C_SCLK,
inout logic 		          		I2C_SDAT,

//////////// ADC //////////
output logic		          		ADC_CS_N,
output logic		          		ADC_SADDR,
output logic		          		ADC_SCLK,
input logic 		          		ADC_SDAT,

//////////// 2x13 GPIO Header //////////
inout logic 		    [12:0]		GPIO_2,
input logic 		     [2:0]		GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN
);			 

logic Laser_signal, Laser_sync, Laser_cod_A, Laser_cod_B, 
		Prop_motor_left_cod_A, Prop_motor_left_cod_B, Prop_motor_right_cod_A, 
		Prop_motor_right_cod_B, UART_TX, UART_RX, UART_DIR,reset,clk;


//output variables///
//roues
logic [15:0] countR,countL;
logic dirR;
logic dirL;
//Lazer
logic [15:0] position,beacon_rising_edge,beacon_falling_edge;
logic position_direction,beacon_detection;

//SPI logic variables
logic [31:0] WriteDataM, DataAdrM;
logic 		 MemWriteM;
logic [31:0] spi_data;
		
assign Laser_signal = GPIO_1[8];
assign Laser_sync = GPIO_1[7];
assign Laser_cod_A = GPIO_1[4];
assign Laser_cod_B = GPIO_1[5];
assign Prop_motor_left_cod_A = GPIO_1[0];
assign Prop_motor_left_cod_B = GPIO_1_IN[0];
assign Prop_motor_right_cod_A = GPIO_1[2];
assign Prop_motor_right_cod_B = GPIO_1[1];
assign UART_TX = GPIO_1[26];
assign UART_RX = GPIO_1[24];
assign UART_DIR = GPIO_1[22];
assign reset = GPIO_0_PI[1];
assign clk = CLOCK_50;

assign LED[4:1]={GPIO_1[0],GPIO_1_IN[0],GPIO_1[2],GPIO_1[1]};

quad_counter quadR(clk,Prop_motor_right_cod_A,Prop_motor_right_cod_B,countR);
quad_counter quadL(clk,Prop_motor_left_cod_A,Prop_motor_left_cod_B,countL);
laserV1 lazer(clk,Laser_signal,Laser_sync,Laser_cod_A,Laser_cod_B,position,position_direction,beacon_rising_edge,beacon_falling_edge,beacon_detection);

//32bit valeur qui contient l'info des roues
logic [31:0] VitesseRoueRL;
logic [31:0] VitesseRoueRLPrev = 32'd0;
assign VitesseRoueRL = {countL,countR};

//32bit valeur qui contient l'info de la tour
logic [31:0] beacon_edge,towerPos;
logic [31:0] beacon_edgePrev,towerPosPrev;
assign beacon_edge = {beacon_rising_edge,beacon_falling_edge}; //adresse 2
initial beacon_edgePrev = 32'd0;
assign towerPos = {16'd0,position}; //adresse 3
initial towerPosPrev = 32'd0;

//adresse du regristre spi qui contient l'info des roues, il est donc necessaire d'en ajouter. on peut aller jusque 0-15. 
logic [31:0] ad1 = 32'd0; //adresse des roues
logic [31:0] ad2 = 32'd1; //adresses de la tour 
logic [31:0] ad3 = 32'd2;

logic writeRoue,writeBeacon_edge,writeTowerPos;


assign writeRoue = ~(VitesseRoueRL == VitesseRoueRLPrev);
assign writeBeacon_edge = ~(beacon_edge == beacon_edgePrev);
assign writeTowerPos = ~(towerPos == towerPosPrev);


assign LED [0] = Laser_sync;

//******** debut du module de transmission d'infos au registre du SPI ******* 
//******** A ne toucher uniquement en cas de rajout de registre a ecrire *****

typedef enum logic [1:0] {S0,S1,S2} statetype;
statetype state, nextstate;
initial state = S0;
initial nextstate = S1;

 always_ff@(posedge clk) begin 
	
	

	if (state==S0) begin
	DataAdrM <= ad1;
	WriteDataM <= VitesseRoueRLPrev;
	state <= nextstate;
	end
	else if(state==S1) begin 
	DataAdrM <= ad2;
	WriteDataM <= beacon_edgePrev;
	state <= nextstate;
	end
	else if(state==S2) begin
	DataAdrM <= ad3;
	WriteDataM <= towerPosPrev;
	state <= nextstate;
	end
	
 end 

always@(state) begin 

	nextstate = state;

	case(state)
		S0: begin if(writeRoue) begin 
			
			MemWriteM = 1'd1;
			VitesseRoueRLPrev = VitesseRoueRL;
			end
			nextstate = S1;
			end
		S1: begin if (writeBeacon_edge) begin 
			
			MemWriteM = 1'd1;
			beacon_edgePrev = beacon_edge;
			end
			nextstate = S2;
			end
		S2: begin if (writeTowerPos) begin 
			
			MemWriteM = 1'd1;
			towerPosPrev = towerPos;
			end
			nextstate = S0;
			end
	endcase

end



// fin de la FSM de transmission d'info au registre du SPI




//=======================================================
//  SPI, NE PAS TOUCHER !!!!! 
//=======================================================

	logic 			spi_clk, spi_cs, spi_mosi, spi_miso;

	spi_slave spi_slave_instance(
		.SPI_CLK    (spi_clk),
		.SPI_CS     (spi_cs),
		.SPI_MOSI   (spi_mosi),
		.SPI_MISO   (spi_miso),
		.Data_WE    (MemWriteM),
		.Data_Addr  (DataAdrM),
		.Data_Write (WriteDataM),
		.Data_Read  (spi_data),
		.Clk        (clk)
	);
	
	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13



	
endmodule



