module lab1 #
(
	parameter WIDTHIN = 16,		// Input format is Q2.14 (2 integer bits + 14 fractional bits = 16 bits)
	parameter WIDTHOUT = 32,	// Intermediate/Output format is Q7.25 (7 integer bits + 25 fractional bits = 32 bits)
	// Taylor coefficients for the first five terms in Q2.14 format
	//parameter [WIDTHIN-1:0] A0 = 16'b01_00000000000000, // a0 = 1
	parameter [WIDTHIN-1:0] A0 = 16'b01_00000000000000, // a0 = 1
	parameter [WIDTHIN-1:0] A1 = 16'b01_00000000000000, // a1 = 1
	parameter [WIDTHIN-1:0] A2 = 16'b00_10000000000000, // a2 = 1/2
	parameter [WIDTHIN-1:0] A3 = 16'b00_00101010101010, // a3 = 1/6
	parameter [WIDTHIN-1:0] A4 = 16'b00_00001010101010, // a4 = 1/24
	parameter [31:0] A5 = 32'b0000000_0000000000000000010001000,  // a5 = 1/120
	
	parameter s0=4'b0000, s1= 4'b0001, s2= 4'b0010, s3=4'b0011, s4=4'b0100, s5=4'b0101, s6=4'b0110, s7=4'b0111, s8=4'b1000, s9=4'b1001, s10 = 4'b1010
)
(
	input clk,
	input reset,	
	
	input i_valid,
	input i_ready,
	output o_valid,
	output o_ready,
	
	
	
	input [WIDTHIN-1:0] i_x,
	output [WIDTHOUT-1:0] o_y
);
//Output value could overflow (32-bit output, and 16-bit inputs multiplied
//together repeatedly).  Don't worry about that -- assume that only the bottom
//32 bits are of interest, and keep them.
logic [WIDTHIN-1:0] x;	// Register to hold input X
logic [WIDTHOUT-1:0] y_Q;	// Register to hold output Y
logic valid_Q1;		// Output of register x is valid
logic valid_Q2;		// Output of register y is valid
logic valid_Q3;	
logic valid_Q4;	
logic valid_Q5;	
logic valid_Q6;	
//logic valid_Q2;	
logic done;
// signal for enabling sequential circuit elements
logic enable;

logic [WIDTHOUT-1:0] y_D;


logic [WIDTHOUT-1:0] in1;
logic [WIDTHIN-1:0] in3;

logic [WIDTHOUT-1:0] out1;
logic [WIDTHOUT-1:0] out2;

logic [WIDTHOUT-1:0] res;



logic [3:0] curr_state;
logic [3:0] next_state;


//// compute y value
mult32x16 Mult0 (.i_dataa(in1), 		.i_datab(x), 	.o_res(out1));
addr32p16 Addr0 (.i_dataa(out1), 	.i_datab(in3), 	.o_res(out2));


assign y_D = out2;


always_comb begin
	// signal for enable
	enable = i_ready;
end



always @(*) begin
next_state = curr_state;

case (curr_state)
	s0: begin done=0; if (enable) begin next_state=s1; end end
	s1: begin done=0; if (enable) begin next_state=s2; end end
	s2: begin done=0; if (enable) begin next_state=s3; end end
	s3: begin done=0; if (enable) begin next_state=s4; end end
	s4: begin done=0; if (enable) begin next_state=s5; end end
	s5: begin done=1; if (enable) begin next_state=s6; end end
   s6: begin done=0; if (enable) begin next_state=s1; end end
	endcase
end

// Infer the registers
always_ff @(posedge clk or posedge reset) begin
	if (reset) begin curr_state <= s0; x <= 0; y_Q <= 0; valid_Q1 <= 1'b0;	valid_Q2 <= 1'b0; valid_Q3 <= 1'b0; valid_Q4 <= 1'b0; valid_Q5 <= 1'b0; valid_Q6 <= 1'b0; end
	//else if (done) begin end 
	else begin
        if (curr_state == s1) begin in1 <= A5; x <= i_x; curr_state <= next_state; in3 <= A4; valid_Q1 <= i_valid;  end
		  if (curr_state == s2) begin in1 <= out2; curr_state <= next_state; in3 <= A3; valid_Q2 <= valid_Q1;  end
		  if (curr_state == s3) begin in1 <= out2; curr_state <= next_state; in3 <= A2; valid_Q3 <= valid_Q2;  end
		  if (curr_state == s4) begin in1 <= out2; curr_state <= next_state; in3 <= A1; valid_Q4 <= valid_Q3;  end
		  if (curr_state == s5) begin in1 <= out2; curr_state <= next_state; in3 <= A0; valid_Q5 <= valid_Q4;  end
        if (curr_state == s6) begin curr_state <= next_state; valid_Q6 <= valid_Q5; y_Q <= y_D;end
	     else curr_state <= next_state;
		 
	end
end
	

// assign outputs
assign o_y = y_Q;
// ready for inputs as long as receiver is ready for outputs */
assign o_ready = done;   		

assign o_valid = valid_Q6 & i_valid;	

endmodule

/*******************************************************************************************/

// Multiplier module for all the remaining 32x16 multiplications
module mult32x16 (
	input  [31:0] i_dataa,
	input  [15:0] i_datab,
	output [31:0] o_res
);

logic [47:0] result;

always_comb begin
	result = i_dataa * i_datab;
end

// The result of Q7.25 x Q2.14 is in the Q9.39 format. Therefore we need to change it
// to the Q7.25 format specified in the assignment by selecting the appropriate bits
// (i.e. dropping the most-significant 2 bits and least-significant 14 bits).
assign o_res = result[45:14];

endmodule

/*******************************************************************************************/

// Adder module for all the 32b+16b addition operations 
module addr32p16 (
	input [31:0] i_dataa,
	input [15:0] i_datab,
	output [31:0] o_res
);

// The 16-bit Q2.14 input needs to be aligned with the 32-bit Q7.25 input by zero padding
assign o_res = i_dataa + {5'b00000, i_datab, 11'b00000000000};

endmodule


/*******************************************************************************************/
