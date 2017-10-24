/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/


module alu(
  input  wire [ 2:0] aluop,
  input  wire [31:0] vsrc1,
  input  wire [31:0] vsrc2,
  output wire [31:0] result,
  output wire        Overflow,
  output wire        CarryOut,
  output wire        Zero
);

wire [31:0] Bin;//add module's input:b 
wire [31:0] add;//result of add module
wire [31:0] cout;//carryout of every bit
wire cin;


assign Bin=(aluop == 3'b111|| aluop == 3'b101 || aluop == 3'b110)?~vsrc2:vsrc2;//if aluop is sub,then B transform to ~B
assign cin=(aluop == 3'b111|| aluop == 3'b101 || aluop == 3'b110)?1:0;//if aluop is sub,then cin=1

add32 a2(vsrc1,Bin,cin,cout[30],cout[31],add);

assign result = (
                (aluop == 3'b000)?vsrc1&vsrc2:
                (aluop == 3'b001)?vsrc1|vsrc2:
                (aluop == 3'b010)?add: // +
                (aluop == 3'b011)?{vsrc2[15:0],16'b0}: //lui
                (aluop == 3'b100)?{vsrc2<<vsrc1[10:6]}://sll
                (aluop == 3'b101)?(({1'b0,vsrc1}<{1'b0,vsrc2})?32'd1:32'd0)://sltiu
                (aluop == 3'b110)?add: // -
                (aluop == 3'b111)?add[31]^Overflow://output slt
                32'bx
                );

assign Overflow = (aluop == 3'b010 || aluop == 3'b110 ||aluop == 3'b111 ||  aluop == 3'b101)?cout[31]^cout[30]:0;//if isn't do add,sub,then Overflow always equal 0
assign Zero =(aluop == 3'b010 || aluop == 3'b110  ||aluop == 3'b111 ||  aluop == 3'b101)?((result == 32'b0)?1:0):0;//if isn't do add,sub,then Zero always equal 0
assign CarryOut =( aluop == 3'b010|| aluop == 3'b110  ||aluop == 3'b111 ||  aluop == 3'b101)?cout[31]^cin:0;//if isn't do add,sub,then CarryOut always equal 0


endmodule //alu

///////////////////////////////////////////////////////////////
// four-bits modules to do 4bits advanced add
///////////////////////////////////////////////////////////////
module add4(  
    input [3:0] a,  
    input [3:0] b,  
    input cin,//lowest carryin
    output cout1,//second highest carryout
    output cout2,//highest carryout
    output [3:0] result
    
    );  
wire [3:0] p;
wire [3:0] g;  
wire [3:0] c;//store every bit's carryout
assign c[0] =cin ;
assign p=a|b;
assign g=a&b;
assign c[1] =g[0]|(p[0]&c[0]) ;
assign c[2] =g[1]|(p[1]&(g[0]|(p[0]&c[0]))) ;
assign cout1 =g[2]|(p[2]&(g[1]|(p[1]&(g[0]|p[0]&c[0])))) ;
assign cout2 =g[3]|(p[3]&(g[2]|(p[2]&(g[1]|(p[1]&(g[0]|(p[0]&c[0]))))))); 
assign result=a+b+cin;
/*
assign result[0]=g[0]^p[0]^c[0];
assign result[1]=g[1]^p[1]^c[1];
assign result[2]=g[2]^p[2]^c[2];
assign result[3]=g[3]^p[3]^cout1;*/

endmodule  

///////////////////////////////////////////////////////////////
//use eight four-bits modules to do 32bits add
///////////////////////////////////////////////////////////////
module add32
(  
    input [31:0] a,  
    input [31:0] b,  
    input cin,
    output cout1,//second highest carryout
    output cout2,//highest carryout
    output [31:0] result
    
    );  
    wire [15:0] cout;

    add4 a3(a[3:0],b[3:0],cin,cout[0],cout[1],result[3:0]);
    add4 a4(a[7:4],b[7:4],cout[1],cout[2],cout[3],result[7:4]);
    add4 a5(a[11:8],b[11:8],cout[3],cout[4],cout[5],result[11:8]);
    add4 a6(a[15:12],b[15:12],cout[5],cout[6],cout[7],result[15:12]);
    add4 a7(a[19:16],b[19:16],cout[7],cout[8],cout[9],result[19:16]);
    add4 a8(a[23:20],b[23:20],cout[9],cout[10],cout[11],result[23:20]);
    add4 a9(a[27:24],b[27:24],cout[11],cout[12],cout[13],result[27:24]);
    add4 a10(a[31:28],b[31:28],cout[13],cout1,cout2,result[31:28]);
	
endmodule