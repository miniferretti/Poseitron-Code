//-------------------------------------------------------------
//
// SPI Module
// 

module spi_slave_mu(
	input  logic		clk,
	input  logic 		SPI_CLK,
	input  logic		SPI_CS,
	input  logic 		SPI_MOSI,
	output logic 		SPI_MISO,

	input  logic [31:0]	speed_FL, speed_RL, trace_R, trace_L,
	input  logic [15:0] red, green, blue, clear,
	input  logic [31:0] dyna_read,
	output logic [31:0]	data_out,
	output logic [31:0] ports_control,
	output logic [31:0] dyna_write,
	output logic [31:0] reg_addr
);

	logic [39:0] SPI_reg;


//--- Input Registers (from Pi to FPGA) ------------------------
	logic [31:0] mosiRAM[15:0];
	logic 		 mosiRAM_we;
    
	assign data_out = mosiRAM[4'h7];
	assign ports_control = mosiRAM[4'h8];
	assign dyna_write = mosiRAM[4'h9];
	assign reg_addr = mosiRAM[4'hA];
		// If needed, we have 16 RAM slots.

	always_ff @(posedge clk) begin
		if (mosiRAM_we) mosiRAM[SPI_reg[35:32]] <= SPI_reg[31:0];
	end
	
//--- Output Registers (from FPGA to Pi) ----------------------
	logic [31:0] misoRAM[15:0];		// RAM Memory
	logic [31:0] misoRAM_read;		// Data to be sent to the RPi

	assign misoRAM_read = misoRAM[SPI_reg[3:0]];
	always_ff @(posedge clk) begin
		// wheel data
		misoRAM[4'h0] <= {speed_FL[15:0], speed_RL[15:0]};

		// right odometer data
		misoRAM[4'h1] <= {trace_R};

		// left odometer data
		misoRAM[4'h2] <= {trace_L};

		//Red color value
		misoRAM[4'h3] <= {16'b0, red};

		//Green color value
		misoRAM[4'h4] <= {16'b0, green};

		//Blue color value
		misoRAM[4'h5] <= {16'b0, blue};

		//Clear color value
		misoRAM[4'h6] <= {16'b0, clear};

		//Dynamixel status values 
		misoRAM[4'hB] <= {dyna_read};

	end

	
//---SPI Sysnchronization -------------------------------------

	logic SPI_CLK_sync;
	logic SPI_CS_sync;

	always_ff @(posedge clk) begin
		SPI_CLK_sync <= SPI_CLK;
		SPI_CS_sync  <= SPI_CS;
	end
	
//--- SPI FSM -------------------------------------------------

	typedef enum logic [1:0] {S0,S1,S2,S3} statetype;
	statetype state, nextstate;
	
	logic [5:0] SPI_cnt;
	logic 		SPI_cnt_reset, SPI_cnt_inc;
	logic			SPI_reg_reset, SPI_reg_shift, SPI_reg_load;	
	logic 		MISO_we, MISO_reset;
	
// State Register & Bit counter & SPI Register & MISO
	
	always_ff @(posedge clk) begin
	
		if (SPI_CS_sync)		state <= S0;
		else 					state <= nextstate;
		
		if (SPI_cnt_reset) 	 	SPI_cnt <= 6'b0;
		else if (SPI_cnt_inc) 	SPI_cnt <= SPI_cnt + 6'b1;
		
		if (SPI_reg_reset) 		SPI_reg <= 40'b0;
		else if (SPI_reg_shift)	SPI_reg <= {SPI_reg[38:0], SPI_MOSI};
		else if (SPI_reg_load)	SPI_reg <= {misoRAM_read, SPI_reg[7:0]};
		
		if (MISO_reset) 		SPI_MISO <= 0;
		else if (SPI_reg_load)	SPI_MISO <= misoRAM_read[31];
		else if (MISO_we)		SPI_MISO <= SPI_reg[39];
 			
	end
	
// Next State Logic

	always_comb begin
	
		// Default value
		nextstate = state;
		SPI_cnt_reset = 0; SPI_cnt_inc = 0;
		SPI_reg_reset = 0; SPI_reg_shift = 0; SPI_reg_load = 0;
		MISO_we = 0; MISO_reset = 0;
		mosiRAM_we = 0;
		
		case (state)
			S0 : if (~SPI_CS_sync) begin 			// negedge of SPI_CS
						SPI_cnt_reset = 1;
						SPI_reg_reset = 1;
						MISO_reset    = 1;
						nextstate = S1;
					end
					
			S1	: if (SPI_CLK_sync) begin			// posedge of SPI_CLK
						SPI_reg_shift = 1;
						SPI_cnt_inc   = 1;
						nextstate = S2;
					end
					
			S2 : if (~SPI_CLK_sync) begin			// negedge of SPI_CLK
						MISO_we = 1;
						if (SPI_cnt == 8) SPI_reg_load = 1;
						if (SPI_cnt == 40) begin
							if (SPI_reg[39] == 1) mosiRAM_we = 1;
							nextstate = S3;
						end
						else nextstate = S1;
					end
					
			S3 : if (SPI_CS_sync) begin 			// posedge of SPI_CS
						nextstate = S0;
					end
		endcase
	end
	
endmodule
