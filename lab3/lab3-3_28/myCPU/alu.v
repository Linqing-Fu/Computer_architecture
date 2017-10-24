module alu(
	input      [31:0] A,
	input      [31:0] B,
	input      [ 3:0] ALUop,
	output            Overflow,
	output            CarryOut,
	output            Zero,
	output  reg   [31:0] Result
);

reg [32 :0] storage_B;
reg         symbol_A;
reg         symbol_B;
reg         symbol_result;
reg [31:0]  temp_result;
reg         is_sub;
reg [63:0]  sra48;
wire        A_and_SB;
wire        A_nor_SB;
wire        neg_R;
reg [32:0] result_33;
/*
assign storage_B = (ALUop == 4'b0010)?{symbol_B,B}:
                   (ALUop == 4'b0011 || ALUop == 4'b0100)?~{symbol_B,B}:
                   B;

assign is_sub    = (ALUop == 4'b0011)?1:0;

assign symbol_A  = (ALUop == 4'b0010 || ALUop == 4'b0011 || ALUop == 4'b0100)?A[31]:0;

assign symbol_B  = (ALUop == 4'b0010 || ALUop == 4'b0011 || ALUop == 4'b0100)?B[31]:0;

assign sra48     = (ALUop == 4'b1001)?{{32{B[31]}},B}>>A:0;

assign result_33 = (ALUop == 4'b0010)?{symbol_A,A} + storage_B:
                    (ALUop == 4'b0011 || ALUop == 4'b0100)?{symbol_A,A} +storage_B + 1:
                    0;

assign symbol_result = (ALUop == 4'b0010 || ALUop == 4'b0011 || ALUop == 4'b0100)?result_33[32]:0;

assign Result    = (ALUop == 4'b0000)?A & storage_B:
                    (ALUop == 4'b0001)?A | storage_B:
                    (ALUop == 4'b0010 || ALUop == 4'b0011)?result_33[31:0]:
                    (ALUop == 4'b0100)?symbol_result:
                    (ALUop == 4'b0101)?~A & ~B:
                    (ALUop == 4'b0110)?A^B:
                    (ALUop == 4'b0111)?B<<A:
                    (ALUop == 4'b1000)?B>>A:
                    (ALUop == 4'b1001)?sra48[31:0]:
                    0;
*/



always@(*) begin
case(ALUop)
//and
4'b0000: begin
        is_sub=0;
        storage_B = B;
        Result=A & storage_B;
        symbol_A=0;
        symbol_B=0;
        symbol_result=0;
        sra48=0;
        end
//or
4'b0001: begin
        is_sub=0;
        storage_B = B;
        Result=A | storage_B;
        symbol_result=0;
        sra48=0;
        end
//add
4'b0010: begin
        is_sub=0;
        symbol_A=A[31];
        symbol_B=B[31];
        storage_B = {symbol_B,B};
        {symbol_result,Result}={symbol_A,A} + storage_B;
        sra48=0;
        end
//sub
4'b0011: begin
        is_sub=1;
        symbol_A=A[31];
        symbol_B=B[31];
        storage_B = ~{symbol_B,B};
        {symbol_result,Result}={symbol_A,A} +storage_B + 1;
        sra48=0;
        end
//
4'b0100:begin
       is_sub=0;
       symbol_A=A[31];
       symbol_B=B[31];
       storage_B = ~{symbol_B,B};
       {symbol_result,temp_result}={symbol_A,A} + storage_B + 1;
       sra48=0;
       if(symbol_result==0)
       Result=0;
       else
       Result=1;
       end
//
4'b0101:begin
        is_sub=0;
		storage_B = B;
		Result=~A & ~B;
		symbol_result=0;
        symbol_A=0;
        symbol_B=0;
        symbol_result=0;
        sra48=0;
        end
//
4'b0110:begin
        is_sub=0;
		storage_B = B;
		Result=A^B;
		symbol_result=0;
        symbol_A=0;
        symbol_B=0;
        symbol_result=0;
        sra48=0;
        end		
//
4'b0111:begin
        is_sub=0;
		storage_B = B;
		Result=B<<A;
		symbol_result=0;
        symbol_A=0;
        symbol_B=0;
        symbol_result=0;
        sra48=0;
        end
//
4'b1000:begin
        is_sub=0;
		storage_B = B;
		Result=B>>A[4:0];
		symbol_result=0;
        symbol_A=0;
        symbol_B=0;
        symbol_result=0;
        sra48=0;
		end
//
4'b1001:begin
        is_sub=0;
		storage_B = B;
		sra48={{32{B[31]}},B}>>A[4:0];		
        Result=sra48[31:0];
		//Result=B>>>A;
		symbol_result=0;
        symbol_A=0;
        symbol_B=0;
        symbol_result=0;
        end	
//default	
default:begin
        is_sub=0;
        storage_B=0;
        symbol_result=0;
        Result=0;
        sra48=0;
        end
endcase
end


assign A_nor_SB = A[31] ^ storage_B[31];
assign A_and_SB = A[31] & storage_B[31];
assign neg_R    = ~Result[31];
assign CarryOut = is_sub^( (neg_R & A_nor_SB) | A_and_SB);
assign Overflow = symbol_result^Result[31];
assign Zero     = ~(|Result);

endmodule
