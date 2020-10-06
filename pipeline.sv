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

	parameter [WIDTHIN-1:0] A5 = 16'b00_00000010001000  // a5 = 1/120

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

logic valid_Q7;



logic [WIDTHOUT-1:0] ans;	







// signal for enabling sequential circuit elements



logic enable;



// Signals for computing the y output



logic [WIDTHOUT-1:0] m0_out; // A5 * x

logic [31:0] m0_res; 

logic [WIDTHOUT-1:0] a0_out; // A5 * x + A4

logic [WIDTHOUT-1:0] m1_out; // (A5 * x + A4) * x

logic [47:0] m1_res;

logic [WIDTHOUT-1:0] a1_out; // (A5 * x + A4) * x + A3

logic [WIDTHOUT-1:0] m2_out; // ((A5 * x + A4) * x + A3) * x

logic [47:0] m2_res; 

logic [WIDTHOUT-1:0] a2_out; // ((A5 * x + A4) * x + A3) * x + A2

logic [WIDTHOUT-1:0] m3_out; // (((A5 * x + A4) * x + A3) * x + A2) * x

logic [47:0] m3_res;

logic [WIDTHOUT-1:0] a3_out; // (((A5 * x + A4) * x + A3) * x + A2) * x + A1

logic [WIDTHOUT-1:0] m4_out; // ((((A5 * x + A4) * x + A3) * x + A2) * x + A1) * x

logic [47:0] m4_res;



logic [WIDTHOUT-1:0] a4_out; // ((((A5 * x + A4) * x + A3) * x + A2) * x + A1) * x + A0

logic [WIDTHOUT-1:0] y_D;







//*************



logic [15:0] A5_reg;

logic [31:0] A4_reg;

logic [31:0] A3_reg;

logic [31:0] A2_reg;

logic [31:0] A1_reg;

logic [31:0] A0_reg;





logic [WIDTHIN-1:0] x1;

logic [WIDTHIN-1:0] x2;

logic [WIDTHIN-1:0] x3;

logic [WIDTHIN-1:0] x4;





logic [WIDTHOUT-1:0] F1;

logic [WIDTHOUT-1:0] F2;

logic [WIDTHOUT-1:0] F3;

logic [WIDTHOUT-1:0] F4;











always_comb begin



		m0_res = A5_reg * x;

		m0_out = {3'b000, m0_res [31:3]};

		a0_out = m0_out + A4_reg;

		

		

		m1_res = F4 * x4;

		m1_out = m1_res[45:14];

		a1_out = m1_out + A3_reg;

		

		

		m2_res = F3 * x3;

		m2_out = m2_res[45:14];

		a2_out = m2_out + A2_reg;



		

		m3_res = F2 * x2;

		m3_out = m3_res[45:14];

		a3_out = m3_out + A1_reg;

		

		m4_res = F1 * x1;

		m4_out = m4_res[45:14];

		a4_out = m4_out + A0_reg ;

		

		enable = i_ready;

			

end





always @(posedge clk) begin



if (enable) begin

		

		/// stage 1

		A5_reg <= A5;

		A4_reg <= {5'b00000, A4, 11'b00000000000};

		

   	F4 <= a0_out;

		x4 <= x;

      

		/// stage 2

		A3_reg <= {5'b00000, A3, 11'b00000000000};

		

		F3 <= a1_out;

		x3 <= x4;

   

    	/// stage 3



		A2_reg <= {5'b00000, A2, 11'b00000000000};

		

		F2 <= a2_out;

		x2 <= x3;

		

	  /// stage 4



		A1_reg <= {5'b00000, A1, 11'b00000000000};

		

   	F1 <= a3_out;

		x1 <= x2;





     ///stage 5



		A0_reg <= {5'b00000, A0, 11'b00000000000};

		

	

end		

    

end



// Infer the registers



//always_comb begin



//enable = i_ready;



//end





always_ff @(posedge clk or posedge reset) begin



	if (reset) begin

		valid_Q1 <= 1'b0;

		valid_Q2 <= 1'b0;

		valid_Q3 <= 1'b0;

		valid_Q4 <= 1'b0;

		valid_Q5 <= 1'b0;

		valid_Q6 <= 1'b0;

		

		

		x <= 0;

		y_Q <= 0;

		



	

	end 

	

	else if (enable) begin



		// propagate the valid value

		valid_Q1 <= i_valid;

		valid_Q2 <= valid_Q1;

		valid_Q3 <= valid_Q2;

		valid_Q4 <= valid_Q3;

		valid_Q5 <= valid_Q4;

		valid_Q6 <= valid_Q5;





		// read in new x value

		x <= i_x;

	

		

		// output computed y value

		y_Q <= y_D;

		





	end



end



assign y_D = a4_out;



// assign outputs



assign o_y = y_Q;



// ready for inputs as long as reiver is ready for outputs */



assign o_ready = i_ready;   		



// the output is valid as long as the corresponding input was valid and 



//	the receiver is ready. If the receiver isn't ready, the computed output



//	will still remain on the register outputs and the circuit will resume



//  normal operation with the receiver is ready again (i_ready is high)*/



assign o_valid = valid_Q6 & i_ready;	



endmodule



