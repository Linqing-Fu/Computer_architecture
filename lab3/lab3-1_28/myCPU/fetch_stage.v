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


 module fetch_stage(
    input  wire        clk,
    input  wire        resetn,

    input  wire [31:0] nextpc,
    input  wire [31:0] inst_sram_rdata,
    //input  wire        pc_enable,

    //input  wire        validin,     //validin
    input  wire        fe_ready_go,     //pipeline control
    input  wire        fe_out_allow,    //allowin from next stage
    output wire        fe_validout,     //validout
    //output wire        fe_allowin,      //serve as fe_stage out_allow

    output reg  [31:0] fe_pc,           //fetch_stage pc
    output wire [31:0] fe_inst          //instr code sent from fetch_stage
);

///////////////////////////////////////////////////pipeline control
reg         fe_valid;
wire        fe_allowin;

assign fe_allowin   = !fe_valid || fe_ready_go && fe_out_allow;
assign fe_validout  = fe_valid && fe_ready_go ;

always @(posedge clk) begin
    if (!resetn) begin
        fe_valid <= 1'b0;        
    end
    if (fe_allowin) begin
        fe_valid <= 1'b1;
    end
end
/////////////////////////////////////////////////////////////////////

assign fe_inst = inst_sram_rdata;

always @(posedge clk) begin
	if (!resetn) begin
		fe_pc <= 32'hbfc00000;
		
	end
	if (fe_allowin) begin //no need for validin
		fe_pc <= nextpc;
	end
end


endmodule //fetch_stage
