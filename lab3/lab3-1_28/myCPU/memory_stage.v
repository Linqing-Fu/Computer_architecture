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


module memory_stage(
    input  wire        clk,
    input  wire        resetn,

    input  wire [ 2:0] exe_out_op,      //control signals used in MEM, WB stages
    input  wire [ 4:0] exe_dest,        //reg num of dest operand
    input  wire [31:0] exe_value,       //alu result from exe_stage or other intermediate 
                                        //value for the following stages

    input  wire [31:0] data_sram_rdata,

    input  wire        exe_validout,     //validin
    input  wire        mem_ready_go,     //pipeline control
    input  wire        mem_out_allow,    //allowin from next stage
    output wire        mem_validout,     //validout
    output wire        mem_allowin,      //serve as exe_stage out_allow

    output wire        mem_out_op,      //control signals used in WB stage
    output reg  [ 4:0] mem_dest,        //reg num of dest operand
    output wire [31:0] mem_value        //mem_stage final result

  //`ifdef SIMU_DEBUG
   ,input  wire [31:0] exe_pc,          //pc @execute_stage
    input  wire [31:0] exe_inst,        //instr code @execute_stage
    output reg  [31:0] mem_pc,          //pc @memory_stage
    output reg  [31:0] mem_inst         //instr code @memory_stage
  //`endif
);

reg  [ 2:0] mem_op_reg;
reg  [31:0] mem_st_value;
reg  [31:0] exe_value_reg;            //store exe_value
wire [31:0] mem_value_wire;          //store exe_value_reg

//////////////////////////////////////////////////////pipeline control
reg         mem_valid;

assign mem_allowin   = !mem_valid || mem_ready_go && mem_out_allow;
assign mem_validout  = mem_valid && mem_ready_go ;

always @(posedge clk) begin
    if (!resetn) begin
        mem_valid <= 1'b0;        
    end
    else if (mem_allowin) begin
        mem_valid <= exe_validout;
    end
end
/////////////////////////////////////////////////////////////////////

//reg num of dest operand
always @(posedge clk) begin
    if (!resetn) begin
        mem_dest <= 0;
    end
    if (exe_validout && mem_allowin) begin
        mem_dest <= exe_dest;
    end
end

//control signals used in  WB stages
always @(posedge clk) begin
    if (!resetn) begin
        mem_op_reg <= 3'b0; 
    end
    if (exe_validout && mem_allowin) begin
        mem_op_reg <= exe_out_op;
    end
end


always @(posedge clk) begin
    if (!resetn) begin
        exe_value_reg <= 32'b0; 
    end
    if (exe_validout && mem_allowin) begin
        exe_value_reg <= exe_value;
    end
end

assign mem_out_op = mem_op_reg[0];  //RegWrite
assign mem_value = (mem_op_reg[2:1] == 2'b01)?data_sram_rdata:exe_value_reg;   //MemtoReg
//assign mem_value_wire = exe_value_reg;



//`ifdef SIMU_DEBUG
always @(posedge clk) begin
    if(!resetn) begin
        mem_pc <= 32'hbfc00000;
    end
    if (exe_validout && mem_allowin) begin
        mem_pc <= exe_pc;
    end
end

always @(posedge clk) begin
    if(!resetn) begin
        mem_inst <= 0;
    end
    if (exe_validout && mem_allowin) begin
        mem_inst <= exe_inst;
    end
end
//`endif



endmodule //memory_stage
