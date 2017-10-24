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


module execute_stage(
    input  wire        clk,
    input  wire        resetn,

    //input  wire [ 2:0] de_ALUctr,
    input  wire [10:0] de_out_op,       //control signals used in EXE, MEM, WB stages
    input  wire [ 4:0] de_dest,         //reg No. of dest operand, zero if no dest
    input  wire [31:0] de_vsrc1,        //value of source operand 1
    input  wire [31:0] de_vsrc2,        //value of source operand 2
    input  wire [31:0] de_st_value,     //value stored to memory

    input  wire        de_validout,     //validin
    input  wire        exe_ready_go,     //pipeline control
    input  wire        exe_out_allow,    //allowin from next stage
    output wire        exe_validout,     //validout
    output wire        exe_allowin,      //serve as de_stage out_allow

    output wire [ 2:0] exe_out_op,      //control signals used in MEM, WB stages
    output reg  [ 4:0] exe_dest,        //reg num of dest operand
    output wire [31:0] exe_value,       //alu result from exe_stage or other intermediate 
                                        //value for the following stages

    output wire        data_sram_en,
    output wire [ 3:0] data_sram_wen,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata 

  //`ifdef SIMU_DEBUG
   ,input  wire [31:0] de_pc,           //pc @decode_stage
    input  wire [31:0] de_inst,         //instr code @decode_stage
    output reg  [31:0] exe_pc,          //pc @execute_stage
    output reg  [31:0] exe_inst         //instr code @execute_stage
  //`endif
);

//reg  [ 2:0] exe_aluctr;       //control of alu
reg  [10:0] exe_op_reg;
reg  [31:0] exe_vsrc1;
reg  [31:0] exe_vsrc2;
reg  [31:0] exe_st_value;    //store to memory


////////////////////////////////////////////////////pipeline control
reg         exe_valid;

assign exe_allowin   = !exe_valid || exe_ready_go && exe_out_allow;
assign exe_validout  = exe_valid && exe_ready_go ;

always @(posedge clk) begin
    if (!resetn) begin
        exe_valid <= 1'b0;        
    end
    else if (exe_allowin) begin
        exe_valid <= de_validout;
    end
end
/////////////////////////////////////////////////////////////////////

//input A of alu
always @(posedge clk) begin
    if (!resetn) begin
        exe_vsrc1 <= 32'b0; 
    end
    if (de_validout && exe_allowin) begin
        exe_vsrc1 <= de_vsrc1;
    end
end

//input B of alu
always @(posedge clk) begin
    if (!resetn) begin
        exe_vsrc2 <= 32'b0; 
    end
    if (de_validout && exe_allowin) begin
        exe_vsrc2 <= de_vsrc2;
    end
end

//control signals used in MEM, WB stages
always @(posedge clk) begin
    if (!resetn) begin
        exe_op_reg <= 11'b0; 
    end
    if (de_validout && exe_allowin) begin
        exe_op_reg <= de_out_op;
    end
end

//reg num of dest operand
always @(posedge clk) begin
    if (!resetn) begin
        exe_dest <= 5'b0; 
    end
    if (de_validout && exe_allowin) begin
        exe_dest <= de_dest;
    end
end

//write into data memory
always @(posedge clk) begin
    if (!resetn) begin
        exe_st_value <= 32'b0; 
    end
    if (de_validout && exe_allowin) begin
        exe_st_value <= de_st_value;
    end
end

assign exe_out_op      = exe_op_reg[2:0];  //{MemtoReg, RegWrite}
assign data_sram_en    = exe_op_reg[7]; //((exe_op_reg == 6'b100011) || (exe_op_reg == 6'b101011))?1'b1:1'b0;
assign data_sram_wen   = exe_op_reg[6:3];  //(exe_op_reg == 6'b101011)?4'b1111:4'b0000;
assign data_sram_addr  = exe_value;
assign data_sram_wdata = exe_st_value;


wire Overflow;
wire CarryOut;
wire Zero;


alu alu0
    (
    .aluop    (exe_op_reg[10:8] ), //I, 3
    .vsrc1    (exe_vsrc1        ), //I, 32
    .vsrc2    (exe_vsrc2        ), //I, 32
    .result   (exe_value        ), //O, 32
    .Overflow (Overflow         ), //O, 1
    .CarryOut (CarryOut         ), //O, 1
    .Zero     (Zero             )  //O, 1 
    );



//`ifdef SIMU_DEBUG
always @(posedge clk) begin
    if(!resetn)
    begin
        exe_pc <= 32'hbfc00000;
    end
    if (de_validout && exe_allowin) begin
        exe_pc <= de_pc;//pc @execute_stage
    end
end


always @(posedge clk) begin
    if(!resetn)
    begin
        exe_inst <= 32'b0;
    end
    if (de_validout && exe_allowin) begin
        exe_inst <= de_inst;//instr code @execute_stage
    end
end
//`endif

endmodule //execute_stage
