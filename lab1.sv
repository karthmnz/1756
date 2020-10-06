module lab1 #
(
	parameter WIDTHIN = 16,		// Input format is Q2.14 (2 integer bits + 14 fractional bits = 16 bits)
	parameter WIDTHOUT = 32,	// Intermediate/Output format is Q7.25 (7 integer bits + 25 fractional bits = 32 bits)
	// Taylor coefficients for the first five terms in Q2.14 format

	parameter [WIDTHIN-1:0] A0 = 16'b01_00000000000000, // a0 = 1
	parameter [WIDTHIN-1:0] A1 = 16'b01_00000000000000, // a1 = 1
	parameter [WIDTHIN-1:0] A2 = 16'b00_10000000000000, // a2 = 1/2
	parameter [WIDTHIN-1:0] A3 = 16'b00_00101010101010, // a3 = 1/6
	parameter [WIDTHIN-1:0] A4 = 16'b00_00001010101010, // a4 = 1/24
	parameter [31:0] A5 = 32'b0000000_0000001000100001111010100,  // a5 = 1/120
	
	parameter s0=4'b000, s1= 4'b001, s2= 4'b010, s3=4'b011, s4=4'b100, s5=4'b101, s6=4'b110 // 4 bit state representation
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

logic [WIDTHIN-1:0] x;	// Register to hold input X
logic [WIDTHOUT-1:0] y_Q;// Register to hold output Y
logic valid_Q1;		
logic valid_Q2;		
logic valid_Q3;	
logic valid_Q4;	
logic valid_Q5;	
logic valid_Q6;	
logic valid_Q7;	

logic done; //mark the end state

// signal for enabling sequential circuit elements
logic enable;

logic [WIDTHOUT-1:0] y_D;


logic [WIDTHOUT-1:0] in1; //multiplication operand 1, second operand is always the x
logic [WIDTHIN-1:0] in3; //addition operand 2
logic [WIDTHOUT-1:0] out1; //multiplication result wire
logic [WIDTHOUT-1:0] out2; //addtion result wire

logic [2:0] curr_state; //register to hold current state
logic [2:0] next_state; //register to hold next state


//// compute y value
mult32x16 Mult0 (.i_dataa(in1), 		.i_datab(x), 	.o_res(out1));     //32x16 multiplier
addr32p16 Addr0 (.i_dataa(out1), 	.i_datab(in3), 	.o_res(out2));   //32x16 adder


assign y_D = out2; //output wire


always @(curr_state, i_ready) begin
next_state = curr_state; 
enable = i_ready;        

if (enable) begin    //when the downstream circuit is ready to accept the output, start processing by trigerring enable
case (curr_state)
	s0: begin  next_state=s1; end 
	s1: begin  next_state=s2; end 
	s2: begin  next_state=s3; end //  reset (s0) -> S1 -> S2 -> S3 -> S4 -> S5 -> S6 (loops back to S1)
	s3: begin  next_state=s4; end 
	s4: begin  next_state=s5; end 
	s5: begin  next_state=s6; end 
   s6: begin  next_state=s1; end 
	endcase
   end
	end

// Infer the registers
always_ff @(posedge clk or posedge reset) begin
	if (reset) begin curr_state <= s0; end  
	//else if (done) begin end 
	else if (enable) begin
	     case(curr_state)
		      s0: begin curr_state <= next_state; x <= 0; y_Q <= 0; valid_Q1 <= 1'b0;	valid_Q2 <= 1'b0; valid_Q3 <= 1'b0; valid_Q4 <= 1'b0; valid_Q5 <= 1'b0; valid_Q6 <= 1'b0;  done<=0; end
				s1: begin in1 <= A5; x <= i_x; curr_state <= next_state; in3 <= A4; valid_Q1 <= i_valid; done<=0;  end // Accept X and compute A5*X + A4
				s2: begin in1 <= out2; curr_state <= next_state; in3 <= A3; valid_Q2 <= valid_Q1; done<=0;  end // ((A5*X + A4) * X) + A3
		      s3: begin in1 <= out2; curr_state <= next_state; in3 <= A2; valid_Q3 <= valid_Q2;  done<=0; end // (((A5*X + A4) * X) + A3)*X) + A2
		      s4: begin in1 <= out2; curr_state <= next_state; in3 <= A1; valid_Q4 <= valid_Q3; done<=0;  end // ..... +A1
		      s5: begin in1 <= out2; curr_state <= next_state; in3 <= A0; valid_Q5 <= valid_Q4; done<=0;  end // ..... +A0 
            s6: begin  curr_state <= next_state; valid_Q6 <= valid_Q5; y_Q <= y_D; done<=1; end //move the result into register and mark done=1 and go back to S1
				
		  endcase
	 end
end
	

// assign outputs
assign o_y = y_Q;
// ready for inputs as long as receiver is ready for outputs */
assign o_ready = done & i_ready;  		//only when done, ready to accept new request from downstream device

assign o_valid = done & valid_Q6 & i_ready;	  //output is valid only when it is done & along with i_valid and downstream device is ready to take the value.

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
