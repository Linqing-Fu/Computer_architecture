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


module writeback_stage(
    input  wire        clk,
    input  wire        resetn,

    input  wire        mem_out_op,      //control signals used in WB stage
    input  wire [ 4:0] mem_dest,        //reg num of dest operand
    input  wire [31:0] mem_value,       //mem_stage final result

    input  wire        mem_validout,    //validin
    input  wire        wb_ready_go,     //pipeline control
    //input  wire        wb_out_allow,    //allowin from next stage
    output wire        wb_validout,     //validout
    output wire        wb_allowin,      //serve as mem_stage out_allow

    output wire        wb_rf_wen,
    output wire [ 4:0] wb_rf_waddr,
    output wire [31:0] wb_rf_wdata 

  //`ifdef SIMU_DEBUG
   ,input  wire [31:0] mem_pc,          //pc @memory_stage
    input  wire [31:0] mem_inst,        //instr code @memory_stage
    output reg  [31:0] wb_pc 
  //`endif
);

reg         wb_op;
reg  [ 4:0] wb_dest;
reg  [31:0] wb_value;

//`ifdef SIMU_DEBUG
reg  [31:0] wb_inst;
//`endif

////////////////////////////////////////////////////pipeline control
reg         wb_valid;

assign wb_allowin   = !wb_valid || wb_ready_go ; //no need foe out_allow
assign wb_validout  = wb_valid && wb_ready_go ;

always @(posedge clk) begin
    if (!resetn) begin
        wb_valid <= 1'b0;        
    end
    else if (wb_allowin) begin
        wb_valid <= mem_validout;
    end
end
////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if (!resetn) begin
        wb_dest <= 5'b0; 
    end
    if (mem_validout && wb_allowin) begin
        wb_dest <= mem_dest;
    end
end


always @(posedge clk) begin
    if (!resetn) begin
        wb_value <= 32'b0; 
    end
    if (mem_validout && wb_allowin) begin
        wb_value <= mem_value;
    end
end


//control signals used in  WB stages
always @(posedge clk) begin
    if (!resetn) begin
        wb_op <= 1'b0; 
    end
    if (mem_validout && wb_allowin) begin
        wb_op <= mem_out_op;
    end
end

assign wb_rf_wen    = wb_op; //RegWrite
assign wb_rf_waddr = wb_dest;
assign wb_rf_wdata = wb_value;

//`ifdef SIMU_DEBUG
always @(posedge clk) begin
    if(!resetn) begin
        wb_pc <= 32'hbfc00000;
    end
    if (mem_validout && wb_allowin) begin
        wb_pc <= mem_pc;
    end
end

always @(posedge clk) begin
    if(!resetn) begin
        wb_inst <= 0;
    end
    if (mem_validout && wb_allowin) begin
        wb_inst <= mem_inst;
    end
end
//`endif



endmodule //writeback_stage
