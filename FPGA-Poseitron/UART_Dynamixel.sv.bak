//----------------------------------------------------------------------
// Module making the interface between the NIOS and the module UART_TXT
// It reads and writes in registers exported from the NIOS
//-----------------------------------------------------------------------
module UART_Dynamixel (
	// NIOS signals
	input				MyNios_Clk,
	input 			    MyNios_Rst,
	input				MyNios_Wr,
	input				MyNios_Rd,
	input  [2:0]	MyNios_Address,
	input  [31:0]	MyNios_WriteData,
	output [31:0]	MyNios_ReadData,
	// exported signals
	input				RXD,
	output			TXD, UART_DIR,
	output    [7:0]      ID_flag
);

logic 			baud_clk, TXD_enable, TXD_done, RXD_enable, RXD_done;
logic [31:0] 	TXD_data_1, TXD_data_2, RXD_data_1, RXD_data_2;

always @(posedge MyNios_Clk)	begin
	// TXD
	if 	    ((MyNios_Rd) & (MyNios_Address == 3'b100)) MyNios_ReadData	    <= {31'b0, TXD_done};
	else if ((MyNios_Wr) & (MyNios_Address == 3'b100)) TXD_enable			<= MyNios_WriteData[0];
	else if ((MyNios_Wr) & (MyNios_Address == 3'b101)) TXD_data_1 			<= MyNios_WriteData;
	else if ((MyNios_Wr) & (MyNios_Address == 3'b110)) TXD_data_2 			<= MyNios_WriteData;
	//RXD
	else if ((MyNios_Rd) & (MyNios_Address == 3'b001)) MyNios_ReadData 	<= RXD_data_1;
	else if ((MyNios_Rd) & (MyNios_Address == 3'b010)) MyNios_ReadData 	<= RXD_data_2;
	else if ((MyNios_Rd) & (MyNios_Address == 3'b000)) MyNios_ReadData 	<= {30'b0, communication_fail, RXD_done};
end

Baudrate_Generator baudgen(MyNios_Clk, MyNios_Rst, baud_clk);
UART_Dynamixel_TXD txd(baud_clk, MyNios_Rst, start_communication, TXD_data_1, TXD_data_2, TXD, UART_DIR, TXD_done);
UART_Dynamixel_RXD rxd(baud_clk, MyNios_Rst, start_communication, RXD, RXD_done, communication_fail, RXD_data_1, RXD_data_2);


logic TXD_enable_prev, start_communication, communication_fail;

assign start_communication = (~TXD_enable_prev) && TXD_enable;
assign ID_flag = TXD_data_1[7:0];

always @(posedge baud_clk)	
	TXD_enable_prev <= TXD_enable;

endmodule 

//------------------------------------------------------------
// Module used to send instruction packets
//------------------------------------------------------------
module UART_Dynamixel_TXD(
	input logic 			clk, reset, start_communication,
	input logic [31:0] 	data1, data2,
	output logic 			TXD, UART_DIR, packet_sent
);

typedef enum logic [2:0] {S0,S1,S2,S3,S4,S5} statetype;
statetype	state, nextstate;

logic 		 start, byte_sent_sig, byte_sent_prev, wait_for_data, byte_ready, byte_sent;
logic [3:0]  cnt_S1;
logic [7:0]  id, length, instruction, P0, P1, P2, checksum, byte_to_send, cnt, size;
logic [71:0] packet_to_send;

assign byte_sent_sig = (~byte_sent_prev) && byte_sent;
assign wait_for_data = cnt_S1 > 4'd10;
assign id 				= data1[7:0];
assign length 			= data1[15:8];
assign instruction 	= data1[23:16];
assign checksum 		= data1[31:24];
assign P0 				= data2[7:0];
assign P1				= data2[15:8];
assign P2				= data2[23:16];

UART_BYTE_TXD ubyte(clk, reset, byte_ready, byte_to_send, byte_sent, TXD);

always_ff @(posedge clk)
	byte_sent_prev <= byte_sent;

always_ff @(posedge clk, posedge reset) begin
	if(reset) state <= S0;
	else		 state <= nextstate;
end

always_comb begin
  case(state)
    S0: if(start_communication) nextstate = S1; //Waiting for the "packet_ready"
		  else nextstate = S0;
    S1: if(wait_for_data) nextstate = S2; 		//Wait for the registers
		  else nextstate = S1;
	 S2: nextstate = S3; 								//Creation of the packet
	 S3: nextstate = S4; 								//Creation of the byte to send
	 S4: if(byte_sent_sig) nextstate = S5; 		//Waiting for the byte to be sent
		  else nextstate = S4;
	 S5: if(packet_sent) nextstate = S0;			//Checking if the whole packet has been sent
		  else nextstate = S3;
    default: nextstate = S0;
  endcase
end

always_ff @(posedge clk) begin
	if(state == S0) begin
		cnt				<= 1'b0;
		cnt_S1			<= 1'b0;
		UART_DIR 		<= 1'b0;
		byte_ready 		<= 1'b0;
		packet_to_send <= 72'b0;
		byte_to_send 	<= 8'b0;
	end
	else if(state == S1) begin
		UART_DIR 	<= 1'b1;
		packet_sent <= 1'b0;
		cnt_S1 		<= cnt_S1 + 4'b1;
	end
	else if(state == S2) begin
		if(length == 8'd4)
			packet_to_send <= {8'h00, checksum, P1, P0, instruction, length, id, 8'hFF, 8'hFF};
		else
			packet_to_send <= {checksum, P2, P1, P0, instruction, length, id, 8'hFF, 8'hFF};
	end
	else if(state == S3) begin
		byte_to_send	<= packet_to_send[7:0];
		packet_to_send <= packet_to_send >> 8;
		byte_ready 		<= 1'b1;
	end
	else if(state == S5) begin
		cnt <= cnt + 4'b1;
		byte_ready <= 1'b0;
		if(cnt > (length + 8'd1))
			packet_sent <= 1'b1;
	end
end

endmodule

//------------------------------------------------------------
// Module used to send a Byte with a start bit and a stop bit
//------------------------------------------------------------
module UART_BYTE_TXD(
	input clk, reset, byte_ready,
	input [7:0] data,
	output byte_sent, TXD
);

logic byte_ready_prev, start, transfer_on;
logic [3:0] cnt;
logic [8:0] packet;

assign start = (~byte_ready_prev)&& byte_ready;
assign byte_sent = (cnt > 4'd8);

always_ff @(posedge clk) 
	byte_ready_prev <= byte_ready;

always_ff @(posedge clk, posedge reset) begin
	if(reset) begin
		transfer_on <= 1'b0;
		cnt 			<= 4'b0;
	end
	else if(start && ~byte_sent && ~transfer_on) begin
		cnt 			<= 4'b0;
		packet 		<= {data, 1'b0};
		transfer_on <= 1'b1;
	end
	else if(transfer_on && ~byte_sent) begin
		TXD 		<= packet[0];
		packet 	<= packet >> 1;
		cnt 		<= cnt + 4'd1;
	end
	else if(transfer_on && byte_sent) begin
	   TXD 			<= 1'b1;
		transfer_on <= 1'b0;
		cnt 			<= 4'b0;
	end
	else begin
		TXD <= 1'b1;
	end
end

endmodule

//--------------------------------------------------------------------------
// UART_Dynamixel_RXD
//--------------------------------------------------------------------------
module UART_Dynamixel_RXD (
	input logic					clk, reset, start_communication, RXD,
	output logic				data_ready, fail_reg,
	output logic [31:0] 		data1, data2
);

typedef enum logic [2:0] {S0,S1,S2,S3} statetype;
statetype	state, nextstate;

logic 		RXD_prev, start, start_byte, end_of_packet, communication_fail;
logic[3:0] 	cnt_byte, cnt_bits;
logic[7:0] 	data, ID, Length, Error, P1, P2, Checksum, cnt_fail;
logic[11:0] RXD_reg;

assign start_byte = (~RXD) && RXD_prev && ((cnt_bits > 4'd7) || (cnt_byte==4'b0));
assign start = (cnt_byte == 4'd3);
assign end_of_packet	= (RXD_reg == 12'hFFF);
assign communication_fail = cnt_fail > 8'd200;

always_ff @(posedge clk, posedge reset) begin
	if(reset) state <= S0;
	else if(communication_fail) state <= S0;
	else 		 state <= nextstate;
end

always_ff @(posedge clk) begin
	if(communication_fail) 	fail_reg <= 1'b1;
	else if(state == S1) 	fail_reg	<= 1'b0;
end

always_ff @(posedge clk) begin
	RXD_prev <= RXD;
	RXD_reg <= {RXD_reg[10:0],RXD};
end

always_ff @(posedge clk) begin
	if(state == S0) cnt_byte <= 4'b0;
	else if(start_byte)  begin
		cnt_byte <= cnt_byte + 4'b1;
		cnt_bits <= 4'b0;
	end
	else cnt_bits <= cnt_bits + 4'b1;
end

always_comb begin
  case(state)
    S0: if(start_communication) nextstate = S1; // Waiting for activation
		  else nextstate = S0;
    S1: if(start) nextstate = S2; 					// Waiting for receiving something
		  else nextstate = S1;
    S2: if(end_of_packet) nextstate = S3; 		// Recording data send by Dynamixel into registers
		  else nextstate = S2;
	 S3: nextstate = S0;									// Creation of the registers
    default: nextstate = S0;
  endcase
end

always_ff @(posedge clk) begin
	if(state == S0) begin
		data_ready	<= 1'b1;
		cnt_fail		<= 8'b0;
	end
	else if(state == S1) begin
		cnt_fail		<= cnt_fail + 8'b1;
		data_ready	<= 1'b0;
		data1 		<= 32'b0;
		data2 		<= 32'b0;
		data 			<= {RXD,data[7:1]};
	end
	else if(state == S2) begin
		data 	<= {RXD,data[7:1]};
	end
	else if(state == S3) begin
		data1 		<= {Checksum, Error, Length, ID};
		data2 		<= {P2,P1};
	end
end

always_ff @(posedge clk) begin
	if(state == S1) begin
		ID 			<= 8'b0;
		Length 		<= 8'b0;
		Error 		<= 8'b0;
		P1 			<= 8'b0;
		P2 			<= 8'b0;
		Checksum 	<= 8'b0;
	end
	else if(cnt_bits == 4'd8) begin
		if(cnt_byte == 4'd3) ID 		 <= data;
		else if(cnt_byte == 4'd4) Length 	 <= data;
		else if(cnt_byte == 4'd5) Error 	 <= data;
		else if(cnt_byte == 4'd6) begin
			if(Length == 8'd2) Checksum <= data;
			else					 P1 <= data;
		end
		else if(cnt_byte == 4'd7) begin
			if(Length == 8'd3) Checksum <= data;
			else				    P2 <= data;
		end
		else if((cnt_byte == 4'd8) && (Length == 8'd4)) Checksum <= data;	
	end
end


endmodule

//--------------------------------------------------------------------------
// Module used to generate a clock of 57.600 [kHz] from a clock of 50 [MHz]
//--------------------------------------------------------------------------
module Baudrate_Generator(input clk, input reset, output baud_clk);

reg[9:0] cnt;

always @(posedge clk or posedge reset) begin
	if (reset) cnt <= 10'b0;
	else begin
		cnt <= cnt + 10'b1;
		if (cnt == 10'd434) begin
			baud_clk <= 1'b1;
		end
		else if(cnt == 10'd868) begin
			baud_clk <= 1'b0;					
			cnt <= 10'b0;
		end
	end 
end

endmodule 