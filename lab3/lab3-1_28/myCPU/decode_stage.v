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


module decode_stage(
    input  wire        clk,
    input  wire        resetn,

    input  wire [31:0] fe_pc,
    input  wire [31:0] fe_inst,
    //input  wire        inst_enable,

    output wire [ 4:0] de_rf_raddr1,
    input  wire [31:0] de_rf_rdata1,
    output wire [ 4:0] de_rf_raddr2,
    input  wire [31:0] de_rf_rdata2,

    input  wire        fe_validout,     //validin
    input  wire        de_ready_go,     //pipeline control
    input  wire        de_out_allow,    //allowin from next stage
    output wire        de_validout,     //validout
    output wire        de_allowin,      //serve as fe_stage out_allow

    output wire        de_br_taken,     //1: branch taken, go to the branch target
    output wire        de_br_is_br,     //1: target is PC+offset
    output wire        de_br_is_j,      //1: target is PC||offset
    output wire        de_br_is_jr,     //1: target is GR value
    output wire [15:0] de_br_offset,    //offset for type "br"
    output wire [25:0] de_br_index,     //instr_index for type "j"
    output wire [31:0] de_br_target,    //target for type "jr"

    output wire [10:0] de_out_op,       //control signals used in EXE, MEM, WB stages {de_ALUctr, data_sram_en, data_sram_wen, MemtoReg, RegWrite}
    output wire [ 4:0] de_dest,         //wire num of dest operand, zero if no dest
    output wire [31:0] de_vsrc1,        //value of source operand 1
    output wire [31:0] de_vsrc2,        //value of source operand 2
    output wire [31:0] de_st_value      //value stored to memory

    //output wire [ 2:0] de_ALUctr           //control of alu

  //`ifdef SIMU_DEBUG
   
    //input  wire [31:0] fe_pc,
    ,output reg  [31:0] de_pc,
    output reg  [31:0] de_inst          //instr code @decode stage
  //`endif
);

reg [31:0] instruction;  //used for decode
reg [31:0] de_pc_use;    //used for debug

//control's relevant wire define
wire [1:0] ALUSrcB;
wire ALUSrcA;
wire PCWriteCondN;
wire PCWriteCondE;
wire data_ram_en;
wire [3:0] data_ram_wen;
wire [1:0] RegDst;
wire [1:0] MemtoReg;
wire [2:0] ALUOp;
wire RegWrite;

//alucontrol
wire [ 2:0] de_ALUctr;

//signextend
wire [31:0] sign_extend;

///////////////////////////////////////////////////pipeline control
reg         de_valid;

assign de_allowin   = !de_valid || de_ready_go && de_out_allow;
assign de_validout  = de_valid && de_ready_go ;

always @(posedge clk) begin
    if (!resetn) begin
        de_valid <= 1'b0;        
    end
    else if (de_allowin) begin
        de_valid <= fe_validout;
    end
end
/////////////////////////////////////////////////////////////////////

always @(posedge clk) begin
    if(!resetn)
    begin
        instruction <= 32'h00000000;
    end
    if (fe_validout && de_allowin) begin
        instruction <= fe_inst;
    end
end

always @(posedge clk) begin
    if(!resetn)
    begin
        de_pc_use <= 32'hbfc00000;
    end
    if (fe_validout && de_allowin) begin
        de_pc_use <= fe_pc;
    end
end






assign sign_extend = {{16{instruction[15]}},instruction[15:0]};//sign extend of 16bits Instruction

assign de_rf_raddr1 = instruction[25:21];     //read1 from reg
assign de_rf_raddr2 = instruction[20:16];     //read2 from reg
assign de_br_taken  = de_br_is_br | de_br_is_j | de_br_is_jr;    //1: branch taken, go to the branch target
assign de_br_is_br  = ((PCWriteCondE & (de_vsrc1 == de_vsrc2)) | (PCWriteCondN & (de_vsrc1 != de_vsrc2)));    //1: target is PC+offset
assign de_br_is_j   = ((instruction[31:26] == 6'b000010) || (instruction[31:26] == 6'b000011))?1'b1:1'b0;   //1: target is PC||offset  jal j
assign de_br_is_jr  = ((instruction[31:26] == 6'b000000) && (instruction[5:0] == 6'b001000))?1'b1:1'b0;   //1: target is GR value

assign de_br_offset = instruction[15:0];      //offset for type "br"
assign de_br_index  = instruction[25:0];     //instr_index for type "j"
assign de_br_target = de_rf_rdata1;      //target for type "jr"


assign de_out_op    = {de_ALUctr,data_ram_en,data_ram_wen,MemtoReg,RegWrite};     //instruction[31:26];
assign de_dest      = (                  //reg num of dest operand, zero if no dest
                        (RegDst == 2'b00)?instruction[20:16]://addiu lui slti sltiu
                        (RegDst == 2'b01)?instruction[15:11]://R-type sll
                        (RegDst == 2'b10)?5'd31://jal
                        5'd0
                        );     
assign de_vsrc1     = (                  //value of source operand 1
                        (ALUSrcA == 1'b0)?de_pc_use://JAL: pc+8
                        ((instruction[31:26] == 6'b000000) && (instruction[5:0] == 6'b000000))?sign_extend://if sll, chose ALUA according to ALUctr
                        (ALUSrcA == 1'b1)?de_rf_rdata1:
                        32'bx
                        );       
assign de_vsrc2     = (                  //value of source operand 2
                        (ALUSrcB == 2'b00)?de_rf_rdata2: //R-type & bne 
                        (ALUSrcB == 2'b01)?8://JAL pc+8
                        (ALUSrcB == 2'b10)?sign_extend:  //LUI SLTI SLTIU ADDIU LW SW
                        (ALUSrcB == 2'b11)?{sign_extend[29:0],2'b0}://BEQ BNE ///////////NOT USE
                        32'bx
                        );        
assign de_st_value  = de_rf_rdata2;   //value stored to memory  sw

Control contrl1(
    .Opcode(instruction[31:26]),
    //.PCSource(PCSource),
    .ALUOp(ALUOp),
    .RegDst(RegDst),
    .ALUSrcA(ALUSrcA),
    .ALUSrcB(ALUSrcB),
    .PCWriteCondN(PCWriteCondN),
    .PCWriteCondE(PCWriteCondE),
    .data_sram_en(data_ram_en),     //exe
    .data_sram_wen(data_ram_wen),   //exe
    .RegWrite(RegWrite),             //wb
    .MemtoReg(MemtoReg)              //wb
    );

ALUcontrol aluctrl(
    .ALUOp(ALUOp),
    .func_tion(instruction[5:0]),
    .ALUctr(de_ALUctr)
);



//`ifdef SIMU_DEBUG
always @(posedge clk) begin
    if(!resetn)
    begin
        de_pc <= 32'hbfc00000;
    end
    if (fe_validout && de_allowin) begin
        de_pc <= fe_pc;
    end
end


always @(posedge clk) begin
    if(!resetn)
    begin
        de_inst <= 32'b0;
    end
    if (fe_validout && de_allowin) begin
        de_inst <= fe_inst;
    end
end
//`endif

endmodule //decode_stage

