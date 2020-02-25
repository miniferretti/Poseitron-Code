module laserV1 (
	input clk,
	input Laser_signal, 
	input Laser_sync, 
	input Laser_cod_A, 
	input Laser_cod_B,
	output [15:0] position,
	output [15:0] beacon_rising_edge,
	output [15:0] beacon_falling_edge);



logic [2:0] quadA_delayed, quadB_delayed;
logic [15:0] countFB;

initial countFB=16'd0;


always_ff @(posedge clk) quadA_delayed <= {quadA_delayed[1:0], Laser_cod_A};
always_ff @(posedge clk) quadB_delayed <= {quadB_delayed[1:0], Laser_cod_B};

logic count_enable; 
logic count_direction;

assign count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
assign count_direction = quadA_delayed[1] ^ quadB_delayed[2];
//always_ff @(posedge quadB) begin 
	//if (quadA) count_direction <= 1'b1;
	//else 		  count_direction <= 1'b0;
//end

always_ff @(posedge clk) begin
	if(Laser_sync) begin
		countFB<=16'd0;
	end
	else begin	
		if(count_enable) begin
			if(count_direction) 	countFB<=countFB+16'd1; 
			else 						countFB<=countFB-16'd1;
		end
		end
	
end
	
	
assign position=countFB;

always_ff@(negedge Laser_signal) begin
	beacon_rising_edge <= countFB;
end 
	
	
	
	
always_ff@(posedge Laser_signal) begin
	beacon_falling_edge <= countFB;
end















endmodule