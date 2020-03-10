


module quad_counter(	input 			clk, 
							input 	 quadA, quadB,
							output 	[15:0] count
							); 
							// counter qui ressort une valeur incrementée si 
							// direction avant sinon decremente direction 
							// arriere. 
							
logic [2:0] quadA_delayed, quadB_delayed;
logic [15:0] countFB;
logic [15:0] countHelper; 
logic [31:0] clockCount;

initial countFB=16'd0;
initial countHelper=16'd0;
initial clockCount=16'd0;

always_ff @(posedge clk) quadA_delayed <= {quadA_delayed[1:0], quadA};
always_ff @(posedge clk) quadB_delayed <= {quadB_delayed[1:0], quadB};

logic count_enable; 
logic count_direction;

assign count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
assign count_direction = quadA_delayed[1] ^ quadB_delayed[2];
//always_ff @(posedge quadB) begin 
	//if (quadA) count_direction <= 1'b1;
	//else 		  count_direction <= 1'b0;
//end

always_ff @(posedge clk) begin
	if(clockCount == 32'd500000) begin
		clockCount<=32'd0;
		countHelper<=countFB;
		countFB<=16'd0;
	end
	else begin	
		if(count_enable) begin
			if(count_direction) 	countFB<=countFB+16'd1; 
			else 						countFB<=countFB-16'd1;
		end
			clockCount<=clockCount+32'd1;
	end	
end
	
assign count=countHelper;
	
endmodule			