module Control(
    input [5:0] Opcode,
    output [1:0] PCSource,
    output [2:0] ALUOp,
    output [1:0] RegDst,
    output ALUSrcA,
    output [1:0] ALUSrcB,
    output PCWriteCondN,//BNE
    output PCWriteCondE,//beq
    output data_sram_en,
    output [3:0] data_sram_wen,
    output RegWrite,
    output [1:0] MemtoReg
    );

assign PCSource = (
                    (Opcode == 6'b000000)?2'b00:
                    (Opcode == 6'b100011)?2'b00:
                    (Opcode == 6'b101011)?2'b00:
                    (Opcode == 6'b000101)?2'b01:
                    (Opcode == 6'b000100)?2'b01:
                    (Opcode == 6'b001001)?2'b00:
                    (Opcode == 6'b001010)?2'b00:
                    (Opcode == 6'b001011)?2'b00:
                    (Opcode == 6'b001111)?2'b00:
                    (Opcode == 6'b000010)?2'b10:
                    (Opcode == 6'b000011)?2'b10:
                    2'b00
                    );

assign ALUOp = (
                    (Opcode == 6'b000000)?3'b010:
                    (Opcode == 6'b100011)?3'b000:
                    (Opcode == 6'b101011)?3'b000:
                    (Opcode == 6'b000101)?3'b001:
                    (Opcode == 6'b000100)?3'b001:
                    (Opcode == 6'b001001)?3'b000:
                    (Opcode == 6'b001010)?3'b011:
                    (Opcode == 6'b001011)?3'b101:
                    (Opcode == 6'b001111)?3'b100:
                    (Opcode == 6'b000010)?3'b000:
                    (Opcode == 6'b000011)?3'b110:
                    3'b000
                    );

assign RegDst = (
                    (Opcode == 6'b000000)?2'b01:
                    (Opcode == 6'b100011)?2'b00:
                    (Opcode == 6'b101011)?2'b00:
                    (Opcode == 6'b000101)?2'b00:
                    (Opcode == 6'b000100)?2'b00:
                    (Opcode == 6'b001001)?2'b00:
                    (Opcode == 6'b001010)?2'b00:
                    (Opcode == 6'b001011)?2'b00:
                    (Opcode == 6'b001111)?2'b00:
                    (Opcode == 6'b000010)?2'b00:
                    (Opcode == 6'b000011)?2'b10:
                    2'b00
                    );

assign ALUSrcA = (
                    (Opcode == 6'b000000)?1'b1:
                    (Opcode == 6'b100011)?1'b1:
                    (Opcode == 6'b101011)?1'b1:
                    (Opcode == 6'b000101)?1'b1:
                    (Opcode == 6'b000100)?1'b1:
                    (Opcode == 6'b001001)?1'b1:
                    (Opcode == 6'b001010)?1'b1:
                    (Opcode == 6'b001011)?1'b1:
                    (Opcode == 6'b001111)?1'b1:
                    (Opcode == 6'b000010)?1'b0:
                    (Opcode == 6'b000011)?1'b0:
                    1'b0
                    );

assign ALUSrcB = (
                    (Opcode == 6'b000000)?2'b00:
                    (Opcode == 6'b100011)?2'b10:
                    (Opcode == 6'b101011)?2'b10:
                    (Opcode == 6'b000101)?2'b00:
                    (Opcode == 6'b000100)?2'b00:
                    (Opcode == 6'b001001)?2'b10:
                    (Opcode == 6'b001010)?2'b10:
                    (Opcode == 6'b001011)?2'b10:
                    (Opcode == 6'b001111)?2'b10:
                    (Opcode == 6'b000010)?2'b00:
                    (Opcode == 6'b000011)?2'b01:
                    2'b00
                    );

assign PCWriteCondN = (
                    (Opcode == 6'b000000)?1'b0:
                    (Opcode == 6'b100011)?1'b0:
                    (Opcode == 6'b101011)?1'b0:
                    (Opcode == 6'b000101)?1'b1:
                    (Opcode == 6'b000100)?1'b0:
                    (Opcode == 6'b001001)?1'b0:
                    (Opcode == 6'b001010)?1'b0:
                    (Opcode == 6'b001011)?1'b0:
                    (Opcode == 6'b001111)?1'b0:
                    (Opcode == 6'b000010)?1'b0:
                    (Opcode == 6'b000011)?1'b0:
                    1'b0
                    );

assign PCWriteCondE = (
                    (Opcode == 6'b000000)?1'b0:
                    (Opcode == 6'b100011)?1'b0:
                    (Opcode == 6'b101011)?1'b0:
                    (Opcode == 6'b000101)?1'b0:
                    (Opcode == 6'b000100)?1'b1:
                    (Opcode == 6'b001001)?1'b0:
                    (Opcode == 6'b001010)?1'b0:
                    (Opcode == 6'b001011)?1'b0:
                    (Opcode == 6'b001111)?1'b0:
                    (Opcode == 6'b000010)?1'b0:
                    (Opcode == 6'b000011)?1'b0:
                    1'b0
                    );

assign data_sram_en = (
                    (Opcode == 6'b000000)?1'b0:
                    (Opcode == 6'b100011)?1'b1:
                    (Opcode == 6'b101011)?1'b1:
                    (Opcode == 6'b000101)?1'b0:
                    (Opcode == 6'b000100)?1'b0:
                    (Opcode == 6'b001001)?1'b0:
                    (Opcode == 6'b001010)?1'b0:
                    (Opcode == 6'b001011)?1'b0:
                    (Opcode == 6'b001111)?1'b0:
                    (Opcode == 6'b000010)?1'b0:
                    (Opcode == 6'b000011)?1'b0:
                    1'b0
                    );

assign data_sram_wen = (
                    (Opcode == 6'b000000)?4'b0000:
                    (Opcode == 6'b100011)?4'b0000:
                    (Opcode == 6'b101011)?4'b1111:
                    (Opcode == 6'b000101)?4'b0000:
                    (Opcode == 6'b000100)?4'b0000:
                    (Opcode == 6'b001001)?4'b0000:
                    (Opcode == 6'b001010)?4'b0000:
                    (Opcode == 6'b001011)?4'b0000:
                    (Opcode == 6'b001111)?4'b0000:
                    (Opcode == 6'b000010)?4'b0000:
                    (Opcode == 6'b000011)?4'b0000:
                    4'b0000
                    );

assign RegWrite = (
                    (Opcode == 6'b000000)?1'b1:
                    (Opcode == 6'b100011)?1'b1:
                    (Opcode == 6'b101011)?1'b0:
                    (Opcode == 6'b000101)?1'b0:
                    (Opcode == 6'b000100)?1'b0:
                    (Opcode == 6'b001001)?1'b1:
                    (Opcode == 6'b001010)?1'b1:
                    (Opcode == 6'b001011)?1'b1:
                    (Opcode == 6'b001111)?1'b1:
                    (Opcode == 6'b000010)?1'b0:
                    (Opcode == 6'b000011)?1'b1:
                    1'b0
                    );

assign MemtoReg = (
                    (Opcode == 6'b000000)?2'b00:
                    (Opcode == 6'b100011)?2'b01:
                    (Opcode == 6'b101011)?2'b00:
                    (Opcode == 6'b000101)?2'b00:
                    (Opcode == 6'b000100)?2'b00:
                    (Opcode == 6'b001001)?2'b00:
                    (Opcode == 6'b001010)?2'b00:
                    (Opcode == 6'b001011)?2'b00:
                    (Opcode == 6'b001111)?2'b00:
                    (Opcode == 6'b000010)?2'b00:
                    (Opcode == 6'b000011)?2'b00:
                    2'b00
                    );


endmodule

module ALUcontrol(
     input [2:0] ALUOp,
     input [5:0] func_tion,
     output [2:0] ALUctr
     );

    assign ALUctr = (
        (ALUOp == 3'b000)?3'b010://LW & SW ADDIU JMP
        (ALUOp == 3'b001)?3'b110://BNE & BEQ
        ((ALUOp == 3'b010) && (func_tion == 6'b100001))?3'b010://ADDU
        ((ALUOp == 3'b010) && (func_tion == 6'b100101))?3'b001://OR
        ((ALUOp == 3'b010) && (func_tion == 6'b000000))?3'b100://SLL
        ((ALUOp == 3'b010) && (func_tion == 6'b101010))?3'b111://SLT
        ((ALUOp == 3'b010) && (func_tion == 6'b001000))?3'b000://JR
        (ALUOp == 3'b011)?3'b111://SLTI
        (ALUOp == 3'b100)?3'b011://LUI
        (ALUOp == 3'b101)?3'b101://SLTIU
        (ALUOp == 3'b110)?3'b010://JAL
        3'bx       
        );   
endmodule 