module laserV1 (
	input clk,
	input Laser_signal, 
	input Laser_sync, 
	input Laser_cod_A, 
	input Laser_cod_B,
	output [15:0] position,
	output position_direction,
	output [15:0] beacon_rising_edge,
	output [15:0] beacon_falling_edge,
	output beacon_detection);
	
	// position
	// Step 1 : detection of edges and direction 
	logic position_direction1;
	logic [15:0] beacon_rising_edge_buf,beacon_falling_edge_buf;
	logic rised,fall;

	always_ff @(posedge Laser_cod_B) begin 
	if (Laser_cod_A) position_direction1 <= 1'd1;
	else 				position_direction1 <= 1'd0;
	end 

	assign position_direction = position_direction1;
	//Step 2 : detect the reset signal laser_sync to reset
	//			  counting taken positions
	
	logic [15:0] position_count; 
	
	initial position_count = 16'd0;
	logic position_reset;

	always_ff@(posedge clk) begin
		if(Laser_sync) begin position_reset <= 1;
		end
		else position_reset <=0;
	end
	 
	
	always_ff @(posedge Laser_cod_B) begin 
		if (position_reset) begin
			position_count <= 16'd0;
		end 
		else begin 
			if (position_direction1) position_count <= position_count + 16'd1;
			else 							position_count <= position_count - 16'd1;
		end 
		
	end 
	
	 assign position = position_count;
	//Step 3 : if beacon detected, set beacon detection on 
	
	assign beacon_detection = ~Laser_signal;
	
	//Step 4 : detect the rising and the falling edge of the beacon and send them back
	
	always_ff@(negedge Laser_signal) begin
	if(rised) begin 
		beacon_rising_edge_buf <= position;	
	end else begin 
	 beacon_rising_edge_buf <= beacon_rising_edge_buf;
	end 
	end 

	assign rised = (beacon_rising_edge_buf==beacon_rising_edge);
	
	always_ff@(posedge Laser_signal) begin
		beacon_falling_edge_buf <= position;
	end

	always_ff@(posedge Laser_sync) begin 
	beacon_rising_edge<=beacon_rising_edge_buf;
	beacon_falling_edge<=beacon_falling_edge_buf;
	end
	
endmodule 