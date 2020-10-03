module lab1 #

(
	parameter WIDTHIN = 16,		// Input format is Q2.14 (2 integer bits + 14 fractional bits = 16 bits)
	parameter WIDTHOUT = 32,	// Intermediate/Output format is Q7.25 (7 integer bits + 25 fractional bits = 32 bits)
	parameter [WIDTHIN-1:0] A0 = 16'b01_00000000000000, // a0 = 1
	parameter [WIDTHIN-1:0] A1 = 16'b01_00000000000000, // a1 = 1
	parameter [WIDTHIN-1:0] A2 = 16'b00_10000000000000, // a2 = 1/2
	parameter [WIDTHIN-1:0] A3 = 16'b00_00101010101010, // a3 = 1/6
	parameter [WIDTHIN-1:0] A4 = 16'b00_00001010101010, // a4 = 1/24
	parameter [WIDTHIN-1:0] A5 = 16'b00_00000010001000  // a5 = 1/120
)
//
(
	input clk,
	input reset,
	

   input i_valid,
	input i_ready,
	output o_valid,
	output o_ready,
	output start, 

	input [WIDTHIN-1:0] i_x,
	output [WIDTHOUT-1:0] o_y
);

logic [WIDTHIN-1:0] x;	// Register to hold input X
logic [WIDTHOUT-1:0] y_Q;	// Register to hold output Y
logic valid_Q1;		// Output of register x is valid
logic valid_Q2;      // Output of register y is valid
logic [WIDTHOUT-1:0] y_D;		

logic [15:0] A5_reg;
logic [31:0] A4_reg;
logic [31:0] A3_reg;
logic [31:0] A2_reg;
logic [31:0] A1_reg;
logic [31:0] A0_reg;

logic [31:0] A0_out;
logic [31:0] A1_out;
logic [31:0] A2_out;
logic [31:0] A3_out;
logic [31:0] A4_out;
logic [15:0] A5_out;

// signal for enabling sequential circuit elements
logic enable;


logic [2:0] curr_count, mux_select, cntr_in;

	always @(posedge clk) begin
		if (enable) begin
		A5_reg <= A5;
		A4_reg <= {5'b00000, A4, 11'b00000000000};
		A3_reg <= {5'b00000, A3, 11'b00000000000};
		A2_reg <= {5'b00000, A2, 11'b00000000000};
		A1_reg <= {5'b00000, A1, 11'b00000000000};
		A0_reg <= {5'b00000, A0, 11'b00000000000};
		end 
		end
		
	

	
	always_comb begin
	
	A5_out = A5_reg;
	A4_out = A4_reg;
	A3_out = A3_reg;
	A2_out = A2_reg;
	A1_out = A1_reg;
	A0_out = A0_reg;
	enable = i_ready;
	end




datapath D (x, clk, reset, A0_out, A1_out, A2_out, A3_out, A4_out, A5_out, ld_mul_op, ld_x, ld_count, ld_mul_res, ld_add_res, count_dec, curr_count, mux_select, cntr_in, done, eqz, Y);
controller C (eqz, start, clk, ld_mul_op, ld_x, ld_count, ld_mul_res, ld_add_res, count_dec, curr_count, mux_select, cntr_in, done);

assign y_D = Y;


// Infer the registers
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			valid_Q1 <= 1'b0;
			valid_Q2 <= 1'b0;
			
			x <= 0;
			y_Q <= 0;
		end else if (enable & ld_x) begin
			// propagate the valid value
			valid_Q1 <= i_valid;
			valid_Q2 <= valid_Q1;
			
			// read in new x value
			x <= i_x;
			
			// output computed y value
			y_Q <= y_D;
		end
	end
   assign start =1;
	// assign outputs
   assign o_y = y_Q;
   

//	// ready for inputs as long as receiver is ready for outputs */
assign o_ready = i_ready;   		
assign o_valid = valid_Q2 & i_ready;	
endmodule



//**********************************************************************************************************************************************************

module datapath (x, clk, reset, A0_out, A1_out, A2_out, A3_out, A4_out, A5_out, ld_mul_op, ld_x, ld_count, ld_mul_res, ld_add_res, count_dec, curr_count, mux_select, cntr_in, done, eqz, Y);

	input clk, reset, ld_mul_op, ld_x, ld_count, ld_mul_res, ld_add_res, count_dec, done;
   input [31:0] A0_out, A1_out, A2_out, A3_out, A4_out;
	input [15:0] A5_out;
	input [15:0] x;
	output [2:0] curr_count;
	input [2:0] cntr_in;
	input [2:0] mux_select;
	output eqz;
	output [31:0] Y;

	

	//logic[31:0] mul_op_reg;
	logic [31:0] W1, W4, W5, W6;
	logic [15:0] W2;
	logic [2:0] W3;
	

	
	//assign curr_count = W3;

	mul_op M1 (.clk(clk), .ld(ld_mul_op), .count(W3), .i_in1(A5_out), .i_in2(W6), .o_out(W1));
	input_mod X (clk, ld_x, x, W2);//
	cntr C (.clk(clk), .ld(ld_count), .dec(count_dec), .i_in(cntr_in), .o_out(W3));//
	mul_mod Mul1 (.i_dataa(W1), .i_datab(W2), .o_res(W4));
	
	mux_mod MU1 (.clk(clk), .i_in0(A0_out), .i_in1(A1_out), .i_in2(A2_out), .i_in3(A3_out), .i_in4(A4_out), .select(mux_select), .o_mux(W5));//
	add_mod Add1 (W4, W5, W6);
	EQZ comp (W3, eqz);
	
	//W3 = cntr_in;
	//eqz = eqz;
	//Y = W6;
   //always_comb begin
	//curr_count = W3;
	//end
	assign curr_count = W3;



endmodule

	
	
module mul_op (clk, ld, count, i_in1, i_in2, o_out);
	input ld, clk;
	input [15:0] i_in1;
	input [31:0] i_in2;
	input [2:0] count;
	output [31:0] o_out;

	logic [31:0] res;

	always @(posedge clk) begin
			if (ld) begin
				if (count==3'b101)begin
				res <= i_in1;
				end
				else res <= i_in2;
				end
			end
		assign o_out = res;

		
endmodule
	
module input_mod (clk, ld, i_x, out);
	input clk, ld;
	input [15:0] i_x;
	output [15:0] out;
   logic [15:0] res;


	always @(posedge clk) begin

	if (ld) res <= i_x;

	end
assign out = res;
endmodule


module cntr (clk, ld, dec, i_in, o_out);
	input clk, ld, dec;
	input [2:0] i_in;
	output [2:0] o_out;
   
	logic [2:0] res;

	always @(posedge clk) begin

	if (ld) res <= 3'b101;
	else if (dec) res <= o_out - 3'b001;
	end
assign o_out = res;
endmodule



module mux_mod (clk, i_in0, i_in1, i_in2, i_in3, i_in4, select, o_mux);
	input clk;
	input[2:0] select;
	input [31:0] i_in0, i_in1, i_in2, i_in3, i_in4;
	output [31:0] o_mux;

	logic [31:0] res;

	always @(select or i_in0 or i_in1 or i_in2 or i_in3 or i_in4)
	begin
		if( select == 3'b000)
			res = i_in0;

		if( select == 3'b001)
			res = i_in1;

		if( select == 3'b010)
			res = i_in2;

		if( select == 3'b011)
			res = i_in3;
		
		if( select == 3'b100)
			res = i_in4;
	end

	assign o_mux=res;
endmodule



// Multiplier module for all the remaining 32x16 multiplications
module mul_mod (
	input  [31:0] i_dataa,
	input  [15:0] i_datab,
	output [31:0] o_res
);

logic [47:0] result;

always_comb begin
	result = i_dataa * i_datab;
end

assign o_res = result[45:14];

endmodule


// Adder module for all the 32b+16b addition operations 
module add_mod (
	input [31:0] i_dataa,
	input [31:0] i_datab,
	output [31:0] o_res
);

// The 16-bit Q2.14 input needs to be aligned with the 32-bit Q7.25 input by zero padding
assign o_res = i_dataa + i_datab;

endmodule

module EQZ (cnt, eqz);
	input [2:0] cnt;
	output eqz;

	assign eqz = (cnt==0);
endmodule

//***********************************************************************************************************************************************
module controller (eqz, start, clk, ld_mul_op, ld_x, ld_count, ld_mul_res, ld_add_res, count_dec, curr_count, mux_select, cntr_in, done);

	input eqz, start, clk;
	input [2:0] curr_count;
	output reg ld_mul_op, ld_x, ld_count, ld_mul_res, ld_add_res, count_dec, done;
	output logic [2:0] mux_select;
	output logic [2:0] cntr_in;

	reg [2:0] state;
	parameter s0=3'b000, s1= 3'b001, s2= 3'b010, s3=3'b011, s4=3'b100;

	always @(posedge clk)
	begin
	case(state)
	s0: state <= s1;
	s1: state <= s2; //push 5 into counter//
	s2: state <= s3; //push count-1 into mux_select
	s3: begin if (eqz) state<= s4; else state<=s2; end
	s4: state <= s0;

	default: state <= s0;
	endcase
	end

	always @(state)
	begin 
	case (state)
	s0: begin ld_mul_op = 0; ld_x = 0; ld_count = 0; ld_mul_res = 0; ld_add_res = 0; count_dec = 0; done=0; end
	s1: begin ld_mul_op = 1; ld_x = 0; ld_count = 1; end
	s2: begin ld_mul_res=1; ld_mul_op = 0; ld_x = 1; ld_count = 0; end
	s3: begin if (curr_count==3'b101) mux_select = curr_count - 3'b001; else mux_select=curr_count; ld_add_res = 1; count_dec=1; end
	s4: begin done=1; ld_add_res = 0; count_dec=0; end

	default: state <= s0;
	endcase
	end

endmodule







	
	
	

