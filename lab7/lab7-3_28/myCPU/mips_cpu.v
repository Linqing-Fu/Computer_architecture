`timescale 10 ns / 1 ns

module mycpu_top(
	input  resetn,
	input  clk,
	input  [5:0] int_n_i,

	output  [31:0] debug_wb_pc,
	output  [ 3:0] debug_wb_rf_wen,
	output  [ 4:0] debug_wb_rf_wnum,
	output  [31:0] debug_wb_rf_wdata,
    
	output inst_sram_en,
	output [ 3:0] inst_sram_wen,
	output [31:0] inst_sram_addr,
	output [31:0] inst_sram_wdata,
	input  [31:0] inst_sram_rdata,
	
	output data_sram_en,
	output [ 3:0] data_sram_wen,
	output [31:0] data_sram_addr,
	output [31:0] data_sram_wdata,
	input  [31:0] data_sram_rdata
);

//used in tlb
wire [31:0] inst_vaddr;
wire [31:0] data_vaddr;
wire [31:0] inst_paddr;
wire [31:0] data_paddr;
wire        tlb_refill_inst;
reg 		tlb_refill_inst_reg;
wire        tlb_refill_data;
wire 		tlb_invalid_inst;
reg 		tlb_invalid_inst_reg;
wire 		tlb_invalid_data;
wire 		tlb_modified;

wire    	index_find;
wire [ 4:0] index_out;
wire [31:0]	Entryhi_out;
wire [11:0] mask;
wire [25:0] Entrylo0_out;
wire [25:0] Entrylo1_out;

//index 0
reg 		cp0_index_p;
reg  [4:0]  cp0_index_in;
wire [31:0] cp0_index;

//entryhi 10
reg  [18:0] cp0_entryhi_vpn2;
reg  [7:0]  cp0_entryhi_asid;
wire [31:0] cp0_entryhi;

//page mask 5
reg  [11:0] cp0_pagemask_mask;
wire [31:0] cp0_pagemask;

//entrylo0
reg  [19:0] cp0_entrylo0_pfn;
reg  [5:0]  cp0_entrylo0_flags;
wire [31:0] cp0_entrylo0;

//entrylo1
reg  [19:0] cp0_entrylo1_pfn;
reg  [5:0]  cp0_entrylo1_flags;
wire [31:0] cp0_entrylo1;

//status 12
reg  		cp0_status_IM7;
reg  		cp0_status_IM6;
reg  		cp0_status_IM5;
reg  		cp0_status_IM4;
reg 		cp0_status_IM3;
reg  		cp0_status_IM2;
reg  		cp0_status_IM1;
reg  		cp0_status_IM0;
reg  		cp0_status_EXL;
reg  		cp0_status_IE ;
//cause 13
reg 		cp0_cause_BD;
reg 		cp0_cause_TI;
reg 		cp0_cause_IP7;
reg 		cp0_cause_IP6;
reg 		cp0_cause_IP5;
reg 		cp0_cause_IP4;
reg 		cp0_cause_IP3;
reg 		cp0_cause_IP2;
reg 		cp0_cause_IP1;
reg 		cp0_cause_IP0;
reg [ 4:0]	cp0_cause_ExcCode;
//hard int
wire [ 5:0]  hw_int_i;
wire Need_int;//use in exe stage
reg  Need_int2;
wire ex_valid;
wire tlb_refill;
wire INT;
//Interrupt
wire count_cmp_eq;
wire timer_int;
wire [5:0] int_pending;
wire  [31:0] cp0_status;
wire  [31:0] cp0_cause; 

//fetch
wire fetch_error;

//decode maintain
wire   Maintain_inst;

//LO HI
reg  [31:0]  LO;
reg  [31:0]  HI;
wire [31:0]  LO_true;
wire [31:0]  HI_true;

//cp0
wire 		cp0_wen;
wire [ 4:0] cp0_waddr;
wire [31:0] cp0_rdata;
wire [31:0] cp0_status_true;
wire [31:0] cp0_cause_true;
wire [31:0] cp0_epc_true;
wire [31:0] cp0_index_true;
wire [31:0] cp0_entryhi_true;
wire [31:0] cp0_entrylo0_true;
wire [31:0] cp0_entrylo1_true;
wire [31:0] cp0_pagemask_true;

reg  [31:0] cp0_Badvaddr;
reg  [31:0] cp0_count;
reg  [31:0] cp0_compare;
reg  [31:0] cp0_epc;

//mul
wire [65:0] mul_result;
wire [32:0] mul_a;
wire [32:0] mul_b;

//div
wire [39:0] s_axis_divisor_tdata;
wire        s_axis_divisor_tready;
reg         s_axis_divisor_tvalid;
wire [39:0] s_axis_dividend_tdata;
wire        s_axis_dividend_tready;
reg         s_axis_dividend_tvalid;
wire [79:0] m_axis_dout_tdata;
wire        m_axis_dout_tvalid;



//register
wire [31:0]  rdata1;  //register read data1
wire [31:0]  rdata2;  //register read data2
wire [31:0]  rdata1_true;
wire [31:0]  rdata2_true;
wire [31:0]  wdata;   //register write data
wire [ 4:0]  waddr;   //register write address

wire [31:0]  data1;       //alu data1
wire [31:0]  data2;       //alu data2
wire [ 3:0]  alu_control; //alu control sigal
wire [31:0]  alu_result1; //alu result: data
wire [31:0]  alu_result2; //alu result: PC	
wire         carryout;    //alu carryout
wire         overflow;    //alu overflow
wire         zero;        //alu zero

wire [31:0]  J_address;     //J: address to be jumped to in the next cycle
reg  [31:0]  PC;
wire [31:0]  PC_next;

wire [31:0]  shift_imm_left_2;  //extended immediate unsigned word shift left 2
wire [31:0]  extension_zero;    //immediate extended with sixteen 0 at the end
wire [27:0]  shift_left_2;      //instruction [25:0] shift left 2
wire [31:0]  sign_extension;    //sign-extended immediate unsigned word
wire [31:0]  zero_extension;    //zero-extended immediate unsigned word
wire         PC_write;           //PC write enable signal
wire [5:0]   op;
wire [5:0]   func;
wire [4:0]   rs;
wire [4:0]   rt;
wire [4:0]   rd;
wire [4:0]   sa;              //6-10 bits in R-type instruction 
wire         rst;             //high level effective
reg          rst2;
reg          rst3;
reg          rst4;
reg          rst5;
wire [31:0]  Instruction;     //instruction SRAM: instruction 
wire [31:0]  Address;         //data SRAM: address
wire [31:0]  Write_data;      //data SRAM: write data
wire [31:0]  Read_data;       //data SRAM: read data 
wire         Mem_en;
wire         Mem_wen;
wire [ 2:0]  load_type;
wire [ 2:0]  store_type;
//debug
wire [31:0] exe_pc;
wire [31:0] mem_pc;
wire [31:0] wb_pc;

//Instructions
wire ADD;
wire ADDI;
wire ADDIU;
wire ADDU;
wire AND;
wire ANDI;
wire BEQ;
wire BGEZ;
wire BGEZAL;
wire BGTZ;
wire BLEZ;
wire BLTZ;
wire BLTZAL;
wire BNE;
wire BREAK;
wire DIV;
wire DIVU;
wire ERET;
wire J;
wire JAL;
wire JALR;
wire JR;
wire LB;
wire LBU;
wire LH;
wire LHU;
wire LUI;
wire LW;
wire LWL;
wire LWR;
wire MFC0;
wire MFHI;
wire MFLO;
wire MTC0;
wire MTHI;
wire MTLO;
wire MULT;
wire MULTU;
wire NOR;
wire OR;
wire ORI;
wire SB;
wire SH;
wire SLL;
wire SLLV;
wire SLT;
wire SLTI;
wire SLTIU;
wire SLTU;
wire SRA;
wire SRAV;
wire SRL;
wire SRLV;
wire SUB;
wire SUBU;
wire SW;
wire SWL;
wire SWR;
wire SYSCALL;
wire TLBP;
wire TLBR;
wire TLBWI;
wire XOR;
wire XORI;
wire Branch;//used in exe_stage indicate whether a branch inst

//Control signals
wire         RegWrite;
wire         wb_RegWrite;
wire [ 2:0]  ALUSrcB;   //exe_alu
wire [ 2:0]  PCSrc;
wire [ 1:0]  RegDst;
wire [ 2:0]  MemtoReg;
wire [31:0]  offset;
wire [ 1:0]  ALUSrcA;
wire         ALUSrcB2;   //pc_alu

/*----------------------------pipe1----------------------------*/
wire        valid_in; 
reg         pipe1_valid;
reg [ 98:0] pipe1_data;
wire        pipe1_allowin;
wire        pipe1_ready_go;
wire        pipe1_to_pipe2_valid;
 
/*----------------------------pipe2----------------------------*/
reg         pipe2_valid;
reg [210:0] pipe2_data;
wire        pipe2_allowin;
wire        pipe2_ready_go;
wire        pipe2_to_pipe3_valid;

/*----------------------------pipe3----------------------------*/
reg         pipe3_valid;
reg [234:0] pipe3_data;///////////////////////////////////////// change 183 -> 215
wire        pipe3_allowin;
wire        pipe3_ready_go;
wire        pipe3_to_pipe4_valid;

/*----------------------------pipe4----------------------------*/
reg         pipe4_valid;
reg [152:0] pipe4_data;
wire        pipe4_allowin;
wire        pipe4_ready_go;

//cp0 signals
assign cp0_wen 		= ~pipe2_data[202] & ~(rst4 & (ex_valid | tlb_refill)) & ~(rst5 & pipe3_data[232]) & MTC0;
assign cp0_waddr 	= rd;




//distinguish instructions
assign op           = pipe1_data[ 31: 26];            //op
assign rs           = pipe1_data[ 25: 21];            //rs
assign rt           = pipe1_data[ 20: 16];            //rt
assign rd           = pipe1_data[ 15: 11];            //rd
assign sa           = pipe1_data[ 10:  6];            //sa
assign func         = pipe1_data[  5:  0];            //func

assign J_address        = {PC[31:28],shift_left_2};
assign shift_left_2     = {pipe1_data[25: 0],2'b00};
assign sign_extension   = {{16{pipe1_data[15]}},pipe1_data[15: 0]};
assign shift_imm_left_2 = sign_extension<<2;
assign zero_extension   = {{16{1'b0}},pipe1_data[15: 0]};
assign extension_zero   = {pipe1_data[15: 0],16'd0};
assign wb_RegWrite      = (pipe4_valid)?pipe4_data[82:82]:0;
 
assign ADD     = (op==6'b000000) & (sa==5'b00000) & (func==6'b100000);
assign ADDI    = (op==6'b001000);
assign ADDIU   = (op==6'b001001);
assign ADDU    = (op==6'b000000) & (sa==5'b00000) & (func==6'b100001);
assign AND     = (op==6'b000000) & (sa==5'b00000) & (func==6'b100100);
assign ANDI    = (op==6'b001100);
assign BEQ     = (op==6'b000100);
assign BGEZ    = (op==6'b000001) & (rt==5'b00001);
assign BGEZAL  = (op==6'b000001) & (rt==5'b10001);
assign BGTZ    = (op==6'b000111) & (rt==5'b00000);
assign BLEZ    = (op==6'b000110) & (rt==5'b00000);
assign BLTZ    = (op==6'b000001) & (rt==5'b00000);
assign BLTZAL  = (op==6'b000001) & (rt==5'b10000);
assign BNE     = (op==6'b000101);
assign BREAK   = (op==6'b000000) & (func==6'b001101);
assign DIV     = (op==6'b000000) & (rd==5'b00000) & (sa==5'b00000) & (func==6'b011010);
assign DIVU    = (op==6'b000000) & (rd==5'b00000) & (sa==5'b00000) & (func==6'b011011);
assign ERET    = ({op,rs,rt,rd,sa,func} == 32'b010000_1000_0000_0000_0000_0000_011000);
assign J       = (op==6'b000010);
assign JAL     = (op==6'b000011);
assign JALR    = (op==6'b000000) & (rt==5'b00000) & (sa==5'b00000) & (func==6'b001001);
assign JR      = (op==6'b000000) & ({rt,rd,sa}==15'b000000000000000) & (func==6'b001000); 
assign LB      = (op==6'b100000);
assign LBU     = (op==6'b100100);
assign LH      = (op==6'b100001);
assign LHU     = (op==6'b100101);
assign LUI     = (op==6'b001111) & (rs==5'b00000);
assign LW      = (op==6'b100011);
assign LWL     = (op==6'b100010);
assign LWR     = (op==6'b100110);
assign MFC0    = (op==6'b010000) & (rs==5'b00000) & (sa==5'b00000) & (func[5:3]==3'b000);
assign MFHI    = (op==6'b000000) & ({rs,rt,sa}==15'b000000000000000) & (func==6'b010000);
assign MFLO    = (op==6'b000000) & ({rs,rt,sa}==15'b000000000000000) & (func==6'b010010);
assign MTC0    = (op==6'b010000) & (rs==5'b00100) & (sa==5'b00000) & (func[5:3]==3'b000);
assign MTHI    = (op==6'b000000) & ({rt,rd,sa} == 15'b000000000000000) & (func==6'b010001);
assign MTLO    = (op==6'b000000) & ({rt,rd,sa} == 15'b000000000000000) & (func==6'b010011);
assign MULT    = (op==6'b000000) & (rd==5'b00000) & (sa==5'b00000) & (func==6'b011000);
assign MULTU   = (op==6'b000000) & (rd==5'b00000) & (sa==5'b00000) & (func==6'b011001);
assign NOR     = (op==6'b000000) & (sa==5'b00000) & (func==6'b100111);
assign OR      = (op==6'b000000) & (sa==5'b00000) & (func==6'b100101);
assign ORI     = (op==6'b001101);
assign SB      = (op==6'b101000);
assign SH      = (op==6'b101001);
assign SLL     = (op==6'b000000) & (rs==5'b00000) & (func==6'b000000);
assign SLLV    = (op==6'b000000) & (sa==5'b00000) & (func==6'b000100);
assign SLT     = (op==6'b000000) & (sa==5'b00000) & (func==6'b101010);
assign SLTI    = (op==6'b001010);
assign SLTIU   = (op==6'b001011);
assign SLTU    = (op==6'b000000) & (sa==5'b00000) & (func==6'b101011);
assign SRA     = (op==6'b000000) & (rs==5'b00000) & (func==6'b000011);
assign SRAV    = (op==6'b000000) & (sa==5'b00000) & (func==6'b000111);
assign SRL     = (op==6'b000000) & (rs==5'b00000) & (func==6'b000010);
assign SRLV    = (op==6'b000000) & (sa==5'b00000) & (func==6'b000110); 
assign SUB     = (op==6'b000000) & (sa==5'b00000) & (func==6'b100010);
assign SUBU    = (op==6'b000000) & (sa==5'b00000) & (func==6'b100011);
assign SW      = (op==6'b101011);
assign SWL     = (op==6'b101010);
assign SWR     = (op==6'b101110);
assign SYSCALL = (op==6'b000000) & (func==6'b001100);
assign TLBP    = (pipe1_data[31:0] == 32'b0100_0010_0000_0000_0000_0000_0000_1000);
assign TLBR    = (pipe1_data[31:0] == 32'b0100_0010_0000_0000_0000_0000_0000_0001);
assign TLBWI   = (pipe1_data[31:0] == 32'b0100_0010_0000_0000_0000_0000_0000_0010);


assign XOR     = (op==6'b000000) & (sa==5'b00000) & (func==6'b100110);
assign XORI    = (op==6'b001110);


assign Maintain_inst = ~(ADD | ADDI | ADDIU | ADDU | AND | ANDI | BEQ | BGEZ | BGEZAL |
						BGTZ | BLEZ | BLTZ | BLTZAL | BNE | BREAK | DIV | DIVU | ERET |
						J | JAL | JALR | JR | LB | LBU | LH | LHU | LUI | LW | LWL | LWR |
						MFC0 | MFHI | MFLO | MTC0 | MTHI | MTLO | MULT | MULTU | NOR | OR |
						ORI | SB | SH | SLL | SLLV | SLT | SLTI | SLTIU | SLTU | SRA | SRAV |
						SRL | SRLV | SUB | SUBU | SW | SWL | SWR | SYSCALL | XOR |XORI | TLBP | TLBR | TLBWI);
assign Branch  = (BEQ | BNE | BGEZ | BGTZ | BLEZ | BLTZ | BGEZAL | BLTZAL | J | JAL | JR | JALR);

always@(posedge clk) begin
    if(rst==1) begin
      	PC <= 32'hbfc00000;
    end
    else if (pipe1_allowin==1) begin
      	PC <= PC_next;
    end
    end

assign fetch_error = (PC[1:0] != 2'b00)?1:0;


//alu source A
assign ALUSrcA[1] = (MFHI || MFLO)?1:0;
assign ALUSrcA[0] = (SLL||SRA||SRL||MFLO) ? 1:0;
assign rdata1_true = (pipe2_valid && (pipe2_data[16:12] == pipe1_data[25:21]) && pipe2_data[17] == 1'b1 && (pipe2_data[11:9] != 3'b1))?wdata:
					(pipe3_valid && (pipe3_data[20:16] == pipe1_data[25:21]) && pipe3_data[21] == 1'b1 && (pipe3_data[15:13] != 3'b1))?pipe3_data[215:184]:
					(pipe4_valid && (pipe4_data[87:83] == pipe1_data[25:21]) && pipe4_data[82] == 1'b1 && (pipe4_data[81:79] != 3'b1))?pipe4_data[119:88]:
					rdata1;

assign HI_true =    (MFHI && pipe2_data[4])?wdata:
					(MFHI && pipe3_data[7])?pipe3_data[215:184]:
					(MFHI && pipe4_data[74])?pipe4_data[119:88]:
					HI;
assign LO_true =  (MFLO && pipe2_data[3])?wdata:
                  (MFLO && pipe3_data[6])?pipe3_data[215:184]:
                  (MFLO && pipe4_data[73])?pipe4_data[119:88]:
                  LO;

assign data1  = (ALUSrcA == 2'b0)? {32{~MTC0}} & rdata1_true:
				(ALUSrcA == 2'b1)? sa:
				(ALUSrcA == 2'b10 && m_axis_dout_tvalid)?m_axis_dout_tdata[31:0]:
				(ALUSrcA == 2'b10 && (m_axis_dout_tvalid == 0))?HI_true:
				(ALUSrcA == 2'b11 && m_axis_dout_tvalid)?m_axis_dout_tdata[71:40]:
                (ALUSrcA == 2'b11 && (m_axis_dout_tvalid == 0))?LO_true:0;
                                
				//({32{((pipe4_data[78] | pipe4_data[77]))}}& m_axis_dout_tdata[31:0])||({32{(pipe4_data[76] | pipe4_data[75])}} & mul_result[63:32]): //div mul-----HI
                //({32{(pipe4_data[78] | pipe4_data[77])}}& m_axis_dout_tdata[71:40])||({32{(pipe4_data[76] | pipe4_data[75])}} & mul_result[31:0]); //div mul----- LO
				//m_axis_dout_tdata[31:0]:m_axis_dout_tdata[71:40];
				
				
				/* && m_axis_dout_tvalid)? :
				(ALUSrcA == 2'b10 && (pipe4_data[76] | pipe4_data[75]))? mul_result[63:32]:
				(ALUSrcA == 2'b11 && m_axis_dout_tvalid)? m_axis_dout_tdata[71:40]:
				(ALUSrcA == 2'b11 && (pipe4_data[76] | pipe4_data[75]))? mul_result[31:0]:
				0;*/
				
				
				
				
				
				//(ALUSrcA == 2'b11)? LO:;

//alu source B
assign ALUSrcB[2] = (MFHI || MFLO || MTHI || MTLO|| MFC0)?1:0;
assign ALUSrcB[1] = (LUI || ORI || XORI || ANDI)?1:0;
assign ALUSrcB[0] = (SW||LW||ADDIU||SLTI||SLTIU||LUI||ADDI||LB || LBU || LH || LHU|| LWL || LWR || SB || SH || SWL || SWR|| MFC0)?1:0;



assign rdata2_true = (pipe2_valid && (pipe2_data[16:12] == pipe1_data[20:16]) && pipe2_data[17] == 1'b1 && (pipe2_data[11:9] != 3'b1))?wdata:
				(pipe3_valid && (pipe3_data[20:16] == pipe1_data[20:16]) && pipe3_data[21] == 1'b1 && (pipe3_data[15:13] != 3'b1))?pipe3_data[215:184]:
				(pipe4_valid && (pipe4_data[87:83] == pipe1_data[20:16]) && pipe4_data[82] == 1'b1 && (pipe4_data[81:79] != 3'b1))?pipe4_data[119:88]:
				rdata2;


assign cp0_status_true = (MFC0 && pipe2_data[194] && rd == pipe2_data[199:195])?wdata:
						 (MFC0 && pipe3_data[223] && rd == pipe3_data[228:224])?pipe3_data[215:184]:
						 cp0_status;
						 //(MFC0 && pipe4_data[67]  && rd == pipe4_data[72:68])?pipe4_data[119:88]:  //all writen in exe_stage, therefore 
						 																			//hazard only occured when de_stage use cp0 regs writen in last inst
						 


assign cp0_cause_true = (MFC0 && pipe2_data[194] && rd == pipe2_data[199:195])?wdata:
						(MFC0 && pipe3_data[223] && rd == pipe3_data[228:224])?pipe3_data[215:184]:
						cp0_cause;
						//(MFC0 && pipe4_data[67]  && rd == pipe4_data[72:68])?pipe4_data[119:88]:
						
						

assign cp0_epc_true = 	(MFC0 && pipe2_data[194] && rd == pipe2_data[199:195])?wdata:
						(MFC0 && pipe3_data[223] && rd == pipe3_data[228:224])?pipe3_data[215:184]:
						cp0_epc;
						//(MFC0 && pipe4_data[67]  && rd == pipe4_data[72:68])?pipe4_data[119:88]:

//((MFC0 || TLBWI) && pipe2_data[207]) || 
assign cp0_index_true = ((MFC0 && pipe2_data[194] && rd == pipe2_data[199:195]) || (TLBWI && pipe2_data[194] && pipe2_data[199:195] == 5'd0))?{27'd0, wdata[4:0]}:
						(MFC0 &&  pipe2_data[208])? {index_find, 26'd0, index_out}:
						((MFC0 && pipe3_data[223] && rd == pipe3_data[228:224]) || (TLBWI && pipe3_data[223] && pipe3_data[228:224] == 5'd0))?{27'd0, pipe3_data[188:184]}:
						cp0_index;

assign cp0_entryhi_true  = 	((MFC0 && pipe2_data[194] && rd == pipe2_data[199:195]) || (TLBWI && pipe2_data[194] && pipe2_data[199:195] == 5'd10) || (TLBP && pipe2_data[194] && pipe2_data[199:195] == 5'd10))?{wdata[31:13], 5'd0, wdata[7:0]}:
							(MFC0 &&  pipe2_data[207])? Entryhi_out:
							((MFC0 && pipe3_data[223] && rd == pipe3_data[228:224]) || (TLBWI && pipe3_data[223] && pipe3_data[228:224] == 5'd10))?{pipe3_data[215:197], 5'd0, pipe3_data[191:184]}:
							cp0_entryhi;

assign cp0_entrylo0_true = 	((MFC0 && pipe2_data[194] && rd == pipe2_data[199:195]) || (TLBWI && pipe2_data[194] && pipe2_data[199:195] == 5'd2))?{6'd0, wdata[25:0]}:
							(MFC0 &&  pipe2_data[207])? {6'd0, Entrylo0_out}:
							((MFC0 && pipe3_data[223] && rd == pipe3_data[228:224]) || (TLBWI && pipe3_data[223] && pipe3_data[228:224] == 5'd2))?{6'd0, pipe3_data[209:184]}:
							cp0_entrylo0;

assign cp0_entrylo1_true = 	((MFC0 && pipe2_data[194] && rd == pipe2_data[199:195]) || (TLBWI && pipe2_data[194] && pipe2_data[199:195] == 5'd3))?{6'd0, wdata[25:0]}:
							(MFC0 &&  pipe2_data[207])? {6'd0, Entrylo1_out}:
							((MFC0 && pipe3_data[223] && rd == pipe3_data[228:224]) || (TLBWI && pipe3_data[223] && pipe3_data[228:224] == 5'd3))?{6'd0, pipe3_data[209:184]}:
							cp0_entrylo1;

assign cp0_pagemask_true = 	((MFC0 && pipe2_data[194] && rd == pipe2_data[199:195]) || (TLBWI && pipe2_data[194] && pipe2_data[199:195] == 5'd5))?{7'd0, wdata[24:13], 13'd0}:
							(MFC0 &&  pipe2_data[207])? {7'd0, mask, 13'd0}:
							((MFC0 && pipe3_data[223] && rd == pipe3_data[228:224]) || (TLBWI && pipe3_data[223] && pipe3_data[228:224] == 5'd5))?{7'd0, pipe3_data[208:197], 13'd0}:
							cp0_pagemask;
/*
assign cp0_index_true = ((MFC0 || TLBWI) && ((pipe2_data[194] && rd == pipe2_data[199:195]) || pipe2_data[207]) )?{27'd0, wdata[4:0]}:
						((MFC0 || TLBWI) && ((pipe3_data[223] && rd == pipe3_data[228:224]) || pipe3_data[233]))?{27'd0, pipe3_data[188:184]}:
						cp0_index;		

assign cp0_entryhi_true =   ((MFC0 || TLBWI) && ((pipe2_data[194] && rd == pipe2_data[199:195]) || pipe2_data[207]) )?{wdata[31:13], 5'd0, wdata[7:0]}:
							((MFC0 || TLBWI) && ((pipe3_data[223] && rd == pipe3_data[228:224]) || pipe3_data[233]))?{pipe3_data[215:197], 5'd0, pipe3_data[191:184]}:
							cp0_entryhi;

assign cp0_entrylo0_true =  ((MFC0 || TLBWI) && ((pipe2_data[194] && rd == pipe2_data[199:195]) || pipe2_data[207]) )?{6'd0, wdata[25:0]}:
							((MFC0 || TLBWI) && ((pipe3_data[223] && rd == pipe3_data[228:224]) || pipe3_data[233]))?{6'd0, pipe3_data[209:184]}:
							cp0_entrylo0;

assign cp0_entrylo1_true =  ((MFC0 || TLBWI) && ((pipe2_data[194] && rd == pipe2_data[199:195]) || pipe2_data[207]) )?{6'd0, wdata[25:0]}:
							((MFC0 || TLBWI) && ((pipe3_data[223] && rd == pipe3_data[228:224]) || pipe3_data[233]))?{6'd0, pipe3_data[209:184]}:
							cp0_entrylo1;

assign cp0_pagemask_true =  ((MFC0 || TLBWI) && ((pipe2_data[194] && rd == pipe2_data[199:195]) || pipe2_data[207]) )?{7'd0, wdata[24:13], 13'd0}:
							((MFC0 || TLBWI) && ((pipe3_data[223] && rd == pipe3_data[228:224]) || pipe3_data[233]))?{7'd0, pipe3_data[208:197], 13'd0}:
							cp0_pagemask;
*/

assign cp0_rdata =  ({32{rd == 5'd0}} & cp0_index_true)
					| ({32{rd == 5'd2}} & cp0_entrylo0_true)
					| ({32{rd == 5'd3}} & cp0_entrylo1_true)
					| ({32{rd == 5'd5}} & cp0_pagemask_true)
					| ({32{rd == 5'd8}} & cp0_Badvaddr)
					| ({32{rd == 5'd9}} & cp0_count)
					| ({32{rd == 5'd10}} & cp0_entryhi_true)
					| ({32{rd == 5'd11}} & cp0_compare)
					| ({32{rd == 5'd12}} & cp0_status_true)
					| ({32{rd == 5'd13}} & cp0_cause_true)
					| ({32{rd == 5'd14}} & cp0_epc_true); 

assign data2      =({32{ALUSrcB==3'd0}} & rdata2_true)
		         | ({32{ALUSrcB==3'd1}} & sign_extension)
		         | ({32{ALUSrcB==3'd2}} & zero_extension)		   
		         | ({32{ALUSrcB==3'd3}} & extension_zero)
		         | ({32{ALUSrcB==3'd4}} & 32'b0)//LUI data
		         | ({32{ALUSrcB==3'd5}} & cp0_rdata);//MFC0

//alu source B2
assign ALUSrcB2 =   (
					(BNE && !zero)
					|| (BEQ && zero) 
					|| (BGEZ && ~rdata1_true[31]) 
					|| (BGTZ && ~rdata1_true[31] && (rdata1_true | 32'b0))
					|| (BLEZ && (rdata1_true[31] || rdata1_true == 32'b0))
					|| (BLTZ && rdata1_true[31])
					|| (BLTZAL && rdata1_true[31])
					|| (BGEZAL && ~rdata1_true[31])
					)?1:0;

assign offset   = (ALUSrcB2==0 || (rst5 & pipe3_data[232]))?4:shift_imm_left_2;//4 or shiftimmediate
//alu control	   
assign alu_control = ({4{AND||ANDI}} & 4'b0000)
                   | ({4{OR || ORI}} & 4'b0001)
				   | ({4{SW || LW || ADDU || ADDIU ||ADD ||ADDI ||MFHI ||MFLO ||MTHI ||MTLO ||LB || LBU || LH || LHU || LWL || LWR || SB || SH || SWL || SWR||MFC0 ||MTC0}} & 4'b0010)
				   | ({4{SLTIU || SLTU || SUB || SUBU}} & 4'b0011)
				   | ({4{SLT || SLTI}} & 4'b0100)
				   | ({4{NOR}} & 4'b0101)
				   | ({4{XOR || XORI}} & 4'b0110)
				   | ({4{SLL || SLLV}} & 4'b0111)
				   | ({4{SRL || SRLV}} & 4'b1000)
				   | ({4{SRA || SRAV}} & 4'b1001);
				
assign inst_sram_en    = 1;
assign inst_sram_wen   = 4'b0000;
assign inst_sram_wdata = 32'b0;
 
assign Mem_en   = (SW || SB || SH || SWL || SWR || LW ||LB || LBU || LH || LHU || LWL || LWR || SB || SH || SWL || SWR)?1:0;
//assign Mem_wen  = (rst3 & ~pipe2_data[202] & ~pipe2_data[201] & ~pipe3_data[230] & (SW || SB || SH || SWL || SWR));
//assign Mem_wen  = (rst3 & ~pipe2_data[202] & ~ex_valid & ~pipe3_data[232] & (SW || SB || SH || SWL || SWR));
assign Mem_wen  = ( ~pipe2_data[202] & ~(rst4 & (ex_valid | tlb_refill)) & ~(rst5 & pipe3_data[232]) & (SW || SB || SH || SWL || SWR));

assign PCSrc[2] = (ERET)?1:0;
assign PCSrc[1] = (JR || JALR)?1:0;
assign PCSrc[0] = (J || JAL || JALR)?1:0;
//to avoid processing the first instruction twice
assign PC_next  = (PC==32'hbfc00000)?PC+4:
			   (rst5 & ex_valid)?32'hbfc00380:
			   (rst5 & tlb_refill)?32'hbfc00200:
			   (rst5 & pipe3_data[232])?PC+4:
              ({32{PCSrc == 3'd0}} & alu_result2) //pc+4  pc+sign_extend(off<<2) 
   			| ({32{PCSrc == 3'd1}} & J_address)  //J address
   			| ({32{PCSrc == 3'd2}} & data1)     //JR address
  			| ({32{PCSrc == 3'd3}} & rdata1_true)       //JARL address
   			| ({32{PCSrc == 3'd4}} & cp0_epc_true); //ERET //| ({32{PCSrc == 3'd5}} & 32'hbfc00380); //SYSCALL
	    	//32'hbfc00380;

assign RegWrite  =   ( ~pipe2_data[202] & ~(rst4 & (ex_valid | tlb_refill)) & ~(rst5 & pipe3_data[232]) &
					(LW |LB |LBU |LH |LHU |LWL |LWR | ADDU | SLTU | NOR | XOR | XORI| SRA | SRAV | SRL | 
					SRLV | SLLV | OR | SLT| SLL | AND | ANDI | ADDIU | JAL | LUI | SLL | SUB | SUBU | SLTI | 
					ADDI | SLTIU | ADD | ORI | BLTZAL | BGEZAL | JALR | MFHI | MFLO | MFC0));

/*
assign RegWrite  =   (rst3 & ~pipe2_data[202] & ~pipe2_data[201] & ~pipe3_data[230] & (LW |LB |LBU |LH |LHU |LWL |LWR | ADDU | SLTU | NOR | XOR | XORI| SRA | SRAV | SRL | 
					SRLV | SLLV | OR | SLT| SLL | AND | ANDI | ADDIU | JAL | LUI | SLL | SUB | SUBU | SLTI | 
					ADDI | SLTIU | ADD | ORI | BLTZAL | BGEZAL | JALR | MFHI | MFLO|MFC0));//~pipe2_data[201] & ~pipe2_data[202] &
*/
assign RegDst[1] = (JAL || BLTZAL || BGEZAL || JALR)?1:0;
assign RegDst[0] = (ADDU || OR || SLT|| SLL || SLLV || SRL || SRLV || SRA || SRAV || ADD || SUB || SUBU || SLTU || 
					AND || NOR || XOR || JALR || MFLO || MFHI)?1:0;

assign waddr=({5{RegDst==2'd0}} & pipe1_data[20:16])//MFC0
           | ({5{RegDst==2'd1}} & pipe1_data[15:11])
		   | ({5{RegDst==2'd2}} & 5'd31)   //JAL BLTZAL BGEZAL 
		   | ({5{RegDst==2'd3}} & pipe1_data[15:11]); //JALR rd<-pc+8
		   
assign MemtoReg[2] = (JAL || BLTZAL || BGEZAL || JALR)?1:0;
assign MemtoReg[1] = (SLTIU ||LUI || SLTU)?1:0;
assign MemtoReg[0] = (SLTIU ||LW || SLTU|| LB ||LBU ||LH ||LHU ||LWL ||LWR)?1:0;

//load type
assign load_type =  ({3{LW}} & 3'b000)
					| ({3{LB}} & 3'b001) | ({3{LBU}} & 3'b010) 
					| ({3{LH}} & 3'b011) | ({3{LHU}} & 3'b100)
				    | ({3{LWL}} & 3'b101) | ({3{LWR}} & 3'b110)
				    | ({3{~LW & ~LB & ~LBU & ~LH & ~LHU & ~LWL & ~LWR}} & 3'b111);
 
//store type
assign store_type = ({3{SW}} & 3'b000)
					| ({3{SB}} & 3'b001)
					| ({3{SH}} & 3'b010)
					| ({3{SWL}} & 3'b011)
					| ({3{SWR}} & 3'b100)
					| ({3{~SW & ~SB & ~SH & ~SWL & ~SWR}} & 3'b101);

wire Overflow_type;
assign Overflow_type = (ADD | ADDI | SUB);

/*----------------------------pipe1----------------------------*/

 //reg valid_in;
 /*always@(posedge clk)
 begin
 if (rst)
  begin
   valid_in<=1'b0;
  end
 else
  begin
   valid_in<=1'b1;
  end
 end*/


assign valid_in=1;
/*
assign pipe1_ready_go=!(( (pipe2_valid && (pipe2_data[16:12])) && (pipe2_data[17:17]) && ((pipe1_data[25:21]==pipe2_data[16:12]) || (pipe1_data[20:16]==pipe2_data[16:12])) ) 
                      || ( (pipe3_valid && (pipe3_data[20:16])) && (pipe3_data[21:21]) && ((pipe1_data[25:21]==pipe3_data[20:16]) || (pipe1_data[20:16]==pipe3_data[20:16])) ) 
                      || ( (pipe4_valid && (pipe4_data[87:83])) && (pipe4_data[82:82]) && ((pipe1_data[25:21]==pipe4_data[87:83]) || (pipe1_data[20:16]==pipe4_data[87:83])) ) );  
*/

assign pipe1_ready_go = !( ~(rst5 & (tlb_refill | ex_valid | pipe3_data[232])) & ( pipe2_valid && pipe2_data[11: 9] == 3'b1 && (pipe2_data[16:12] == rs || pipe2_data[16:12] == rt))
						|| ~(rst5 & pipe3_data[232]) & ( pipe3_valid && pipe3_data[15:13] == 3'b1 && (pipe3_data[20:16] == rs || pipe3_data[20:16] == rt))
						||( pipe4_valid && pipe4_data[81:79] == 3'b1 && (pipe4_data[87:83] == rs || pipe4_data[87:83] == rt)) 
						|| ~(rst5 & pipe3_data[232]) & ( pipe2_valid && (pipe2_data[6:5] != 2'b0))
						|| ~(rst5 & pipe4_data[152]) & ( pipe3_valid && (pipe3_data[9:8] != 2'b0))
						||  ( pipe4_valid && (pipe4_data[76:75] != 2'b0))
						//||( (MFHI || MFLO ) && ( pipe2_valid && (pipe2_data[8:5] != 4'b0)))
						//||( (MFHI || MFLO ) && ( pipe3_valid && (pipe3_data[11:8] != 4'b0)))
						//||( (MFHI || MFLO ) && ( pipe4_valid && (pipe4_data[78:75] != 4'b0)))
						||~(rst5 & pipe3_data[232]) & ( rst3  && (pipe2_data[8:7]!=2'b0) && (m_axis_dout_tvalid == 0)));
						//||( rst3  && (DIV | DIVU ) && (m_axis_dout_tvalid == 0)));
						//||( (DIV  || DIVU)  && (m_axis_dout_tvalid == 0)));
						//||ADDIU || ADDU || AND || ANDI || BEQ || BGEZ || BGEZAL || BGTZ || BLEZ || BLTZ 
                                                    // || BLTZAL || BNE || BREAK || ERET || J || JAL || JALR || JR || LB || LBU || LH || LHU || LUI || LW || LWL || LWR 
                                                     //|| MFC0 || MFHI || MFLO || MTC0 || MTHI || MTLO || NOR || OR || ORI || SB || SH || SLL || SLLV || SLT || SLTI
                                                     //|| SLTIU || SLTU || SRA || SRAV || SRL || SRLV || SUB || SUBU || SW || SWL || SWR || XOR || XORI 

//assign pipe1_ready_go=!(( (pipe1_valid && (waddr)) && (RegWrite) && ((Instruction[25:21]==waddr) || (Instruction[20:16]==waddr)) ) 
  //                    || ( (pipe2_valid && (pipe2_data[16:12])) && (pipe2_data[17:17]) && ((Instruction[25:21]==pipe2_data[16:12]) || (Instruction[20:16]==pipe2_data[16:12])) ) 
    //                  || ( (pipe3_valid && (pipe3_data[20:16])) && (pipe3_data[21:21]) && ((Instruction[25:21]==pipe3_data[20:16]) || (Instruction[20:16]==pipe3_data[20:16])) ) );
                      
 assign pipe1_allowin        = !pipe1_valid || pipe1_ready_go && pipe2_allowin;
 assign pipe1_to_pipe2_valid = pipe1_valid && pipe1_ready_go;
 assign Instruction          = inst_sram_rdata;

always @(posedge clk) begin
     if(rst) begin
	     pipe1_valid <= 1'b0;
	 end
	 else if(pipe1_allowin) begin
	     pipe1_valid <= valid_in;
	 end
	 if(valid_in && pipe1_allowin) begin
	 	 pipe1_data[98]    <= tlb_invalid_inst_reg;//tlb invalid_inst_errot
	 	 pipe1_data[97]    <= tlb_refill_inst_reg ;//tlb_refill_inst_error;   
	 	 pipe1_data[96]    <= fetch_error;
	     pipe1_data[95:64] <= PC;
	     pipe1_data[63:32] <= PC+8;
		 pipe1_data[31: 0] <= Instruction;
	 end
end 



wire [31:0] de_pc;
assign de_pc = pipe1_data[95:64];

/*----------------------------pipe2----------------------------*/
 assign pipe2_ready_go       = pipe2_valid;
 assign pipe2_allowin        = !pipe2_valid || (pipe2_ready_go && pipe3_allowin);
 assign pipe2_to_pipe3_valid = pipe2_valid && pipe2_ready_go;
 
 wire exe_overflow_type;
 assign exe_overflow_type = pipe2_data[205];
 always @(posedge clk) begin
     if(rst) begin
	     pipe2_valid <= 1'b0;
	   end
	 else if(pipe2_allowin) begin
	     pipe2_valid<=pipe1_to_pipe2_valid;
	   end
	 if(pipe1_to_pipe2_valid && pipe2_allowin) begin
		 pipe2_data[210]     <= pipe1_data[98];// tlb invalid_inst_errot
	 	 pipe2_data[209]     <= pipe1_data[97];//tlb_refill_inst_error; 
	 	 pipe2_data[208]     <= TLBP;
	 	 pipe2_data[207]     <= TLBR;
	 	 pipe2_data[206]     <= pipe1_data[96];//fetch_error
	     pipe2_data[205]     <= Overflow_type;
	 	 pipe2_data[204]     <= Maintain_inst;
	 	 pipe2_data[203]     <= BREAK;
	 	 pipe2_data[202]     <= ERET;
	 	 pipe2_data[201]     <= SYSCALL;
	     pipe2_data[200]     <= Branch;
	 	 pipe2_data[199:195] <= cp0_waddr;
	 	 pipe2_data[194] 	 <= cp0_wen;
	     pipe2_data[193:191] <= store_type;
	 	 pipe2_data[190:188] <= load_type; 
	 	 pipe2_data[187:184] <= 4'b0; 
	     pipe2_data[183:152] <= rdata2_true;            //rdata2 for SW and LW
	     pipe2_data[151:120] <= pipe1_data[95:64]; //PC
	     pipe2_data[119: 88] <= pipe1_data[63:32]; //PC+8
		 pipe2_data[ 87: 56] <= data1;             //ALU1 data1
		 pipe2_data[ 55: 24] <= data2;              //ALU1 data2
		 pipe2_data[ 23: 20] <= alu_control;
		 pipe2_data[ 19]     <= Mem_en ;
		 pipe2_data[ 18]     <= Mem_wen ;   //(rst3 & (~pipe3_data[230] & ~pipe3_data[229]) & Mem_wen ); //1 bit 
		 pipe2_data[ 17]     <= RegWrite;// ~(exe_overflow_type & overflow) 
		 pipe2_data[ 16: 12] <= waddr;
		 pipe2_data[ 11:  9] <= MemtoReg;
		 pipe2_data[  8:  5] <= {DIV, DIVU, MULT, MULTU} & {4{~(rst4 & (ex_valid | tlb_refill) )}} & {4{~(rst5 & pipe3_data[232])}};
		 pipe2_data[  4:  3] <= {MTHI, MTLO} & {2{~(rst4 & (ex_valid | tlb_refill))}}  & {2{~(rst5 & pipe3_data[232])}};
		 pipe2_data[  2:  1] <= 2'b0;//pipe1_data[1:0];  //offset[1:0]
		 pipe2_data[0]       <= 1'b0;
	 end 
 end


wire   debug_Memwen;
wire   debug_RegWrite;
wire   debug_mem_syscall;
wire   debug_mem_eret;
assign debug_Memwen = pipe2_data[18];
assign debug_RegWrite = pipe2_data[17];
assign debug_mem_syscall = pipe3_data[229];
assign debug_mem_eret     = pipe3_data[230];
 

assign exe_pc = pipe2_data[151:120];

assign wdata=({32{pipe2_data[11: 9]==3'd0}} & alu_result1)//ALUresult1   
           //| ({32{pipe2_data[11: 9]==3'd1}} & Read_data)
		   | ({32{pipe2_data[11: 9]==3'd2}} & pipe2_data[55:24])
		   | ({32{pipe2_data[11: 9]==3'd3}} & carryout)//slt(Carryout)
		   | ({32{pipe2_data[11: 9]==3'd4}} & pipe2_data[119:88]);//PC+8


assign Address       =  alu_result1;


wire [31:0] SB_wdata;
wire [31:0] SH_wdata;
wire [31:0] SWL_wdata;
wire [31:0] SWR_wdata;

assign SB_wdata      =  (alu_result1[1:0] == 2'b00)?{24'b0,pipe2_data[159:152]}:
						(alu_result1[1:0] == 2'b01)?{16'b0,pipe2_data[159:152],8'b0}:
						(alu_result1[1:0] == 2'b10)?{8'b0,pipe2_data[159:152],16'b0}:
						(alu_result1[1:0] == 2'b11)?{pipe2_data[159:152],24'b0}:
						32'b0;

assign SH_wdata      =  (alu_result1[1:0] == 2'b00)?{16'b0,pipe2_data[167:152]}:
						(alu_result1[1:0] == 2'b10)?{pipe2_data[167:152],16'b0}:
						32'b0; 

assign SWL_wdata     =  (alu_result1[1:0] == 2'b00)?{24'b0,pipe2_data[183:176]}:
						(alu_result1[1:0] == 2'b01)?{16'b0,pipe2_data[183:168]}:
						(alu_result1[1:0] == 2'b10)?{8'b0,pipe2_data[183:160]}:
						(alu_result1[1:0] == 2'b11)?pipe2_data[183:152]:
						32'b0;

assign SWR_wdata     =  (alu_result1[1:0] == 2'b00)?pipe2_data[183:152]:
						(alu_result1[1:0] == 2'b01)?{pipe2_data[175:152],8'b0}:
						(alu_result1[1:0] == 2'b10)?{pipe2_data[167:152],16'b0}:
						(alu_result1[1:0] == 2'b11)?{pipe2_data[159:152],24'b0}:
						32'b0;

assign Write_data    =  (pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b000)?pipe2_data[183:152]://[183:152]->[55:24]
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b001)?SB_wdata:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b010)?SH_wdata:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b011)?SWL_wdata:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b100)?SWR_wdata:
						32'b0;


assign data_sram_en  =  pipe2_data[19] & ~ex_valid & ~tlb_refill;

wire [3:0] SB_wen;
wire [3:0] SH_wen;
wire [3:0] SWL_wen;
wire [3:0] SWR_wen;

assign SB_wen        =  (alu_result1[1:0] == 2'b00)?4'b0001:
						(alu_result1[1:0] == 2'b01)?4'b0010:
						(alu_result1[1:0] == 2'b10)?4'b0100:
						(alu_result1[1:0] == 2'b11)?4'b1000:
						4'b0000;

assign SH_wen        =  (alu_result1[1:0] == 2'b00)?4'b0011:
						(alu_result1[1:0] == 2'b10)?4'b1100:
						4'b0000;

assign SWL_wen       =  (alu_result1[1:0] == 2'b00)?4'b0001:
						(alu_result1[1:0] == 2'b01)?4'b0011:
						(alu_result1[1:0] == 2'b10)?4'b0111:
						(alu_result1[1:0] == 2'b11)?4'b1111:
						4'b0000;

assign SWR_wen       =  (alu_result1[1:0] == 2'b00)?4'b1111:
						(alu_result1[1:0] == 2'b01)?4'b1110:
						(alu_result1[1:0] == 2'b10)?4'b1100:
						(alu_result1[1:0] == 2'b11)?4'b1000:
						4'b0000;

assign data_sram_wen =  (ex_valid | tlb_refill)?4'b0000:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b000)?4'b1111:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b001)?SB_wen:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b010)?SH_wen:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b011)?SWL_wen:
						(pipe2_data[18] == 1'b1 && pipe2_data[193:191] == 3'b100)?SWR_wen:
						4'b0000;

//interrupt

assign cp0_index  = {cp0_index_p, 26'd0, cp0_index_in};

assign cp0_entrylo0 = {6'd0, cp0_entrylo0_pfn, cp0_entrylo0_flags};

assign cp0_entrylo1 = {6'd0, cp0_entrylo1_pfn, cp0_entrylo1_flags};

assign cp0_pagemask = {7'd0, cp0_pagemask_mask, 13'd0};

assign cp0_entryhi  = {cp0_entryhi_vpn2, 5'd0, cp0_entryhi_asid};

assign cp0_status = {9'd0, 1'd1, 6'd0, cp0_status_IM7, cp0_status_IM6, cp0_status_IM5,
					cp0_status_IM4, cp0_status_IM3, cp0_status_IM2, cp0_status_IM1,
					cp0_status_IM0, 6'd0, cp0_status_EXL, cp0_status_IE};





assign cp0_cause  = {cp0_cause_BD, cp0_cause_TI, 14'd0, cp0_cause_IP7, cp0_cause_IP6,
					cp0_cause_IP5, cp0_cause_IP4, cp0_cause_IP3, cp0_cause_IP2,
					cp0_cause_IP1, cp0_cause_IP0, 1'b0, cp0_cause_ExcCode, 2'd0};




assign count_cmp_eq = cp0_compare == cp0_count;
assign timer_int    = cp0_cause_TI;//cause_TI
assign hw_int_i     = ~int_n_i;
assign int_pending[5]   = hw_int_i[5] | timer_int;
assign int_pending[4:0] = hw_int_i[4:0]; 



//CP0 reg
reg count_add_en;
always @(posedge clk)
	count_add_en <= (rst || rst3 & (pipe2_data[194] && pipe2_data[199:195] == 5'd9))?1'b0 : ~count_add_en;


assign INT =   ((cp0_cause_IP7 & cp0_status_IM7) |
				(cp0_cause_IP6 & cp0_status_IM6) |
				(cp0_cause_IP5 & cp0_status_IM5) |
				(cp0_cause_IP4 & cp0_status_IM4) |
				(cp0_cause_IP3 & cp0_status_IM3) |
				(cp0_cause_IP2 & cp0_status_IM2) |
				(cp0_cause_IP1 & cp0_status_IM1) |
				(cp0_cause_IP0 & cp0_status_IM0)) & cp0_status_IE & ~cp0_status_EXL;

assign ex_valid = 	~cp0_status_EXL &
					(INT | pipe2_data[201] | pipe2_data[203] | pipe2_data[204] | //syscall break maintain_inst
					(pipe2_data[205] & overflow) | //|//integer overflow
					((pipe2_data[190:188] == 3'd3) & (alu_result1[0] != 1'b0)) |//lh 
					((pipe2_data[190:188] == 3'd4) & (alu_result1[0] != 1'b0)) |//lhu
					((pipe2_data[190:188] == 3'd0) & (alu_result1[1:0] != 2'b00)) | //LW
					((pipe2_data[193:191] == 3'd2) & (alu_result1[0] != 1'd0)) |//sh
					((pipe2_data[193:191] == 3'd0) & (alu_result1[1:0] != 2'b00))  |//sw
					(pipe2_data[206]) |//fetch_error
					((pipe2_data[193:191] != 3'b101|pipe2_data[190:188] != 3'b111) & tlb_invalid_data == 1'b1)|//tlb invalid load/store 
					(pipe2_data[210]) |//tlb invalid fetch
					(pipe2_data[193:191] != 3'b101 & tlb_modified == 1'b1)//store 's tlb_modified
					); 

//wire debug_fetch_error = pipe2_data[206];

assign tlb_refill = ~cp0_status_EXL & ((pipe2_data[193:191] != 3'b101 & tlb_refill_data == 1'b1) |
					(pipe2_data[190:188] != 3'b111 & tlb_refill_data == 1'b1) |
					(pipe2_data[209]));//tlb_fetch error
wire debug_refill_fetch;
assign debug_refill_fetch = pipe2_data[209];
wire debug_tlb_refill2;
assign debug_tlb_refill2 = (pipe2_data[193:191] != 3'b101 & tlb_refill_data == 1'b1);
wire debug_tlb_refill3;
assign debug_tlb_refill3 = (pipe2_data[190:188] != 3'b111 & tlb_refill_data == 1'b1);

wire debug_fetch_error;
assign debug_fetch_error = pipe2_data[209];
					  
//wire   int_ex_valid;
//assign int_ex_valid = INT | ex_valid;


//index      reg:0
always @(posedge clk) begin
	if (rst) begin
		cp0_index_in <= 5'd0;
		cp0_index_p  <= 1'd0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd0) begin
		cp0_index_in <= alu_result1[4:0];
	end
	else if (pipe2_data[208]) begin //tlbp not found 
		cp0_index_in <= index_out;
		if (index_find) begin
			cp0_index_p <= 1'd1;
		end
		else begin
			cp0_index_p <= 1'd0;
		end
	end
	
end

//entrylo0      reg:2
always @(posedge clk) begin
	if (rst) begin
		cp0_entrylo0_pfn    <= 20'd0;
		cp0_entrylo0_flags  <= 6'd0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd2) begin
		cp0_entrylo0_pfn   <= alu_result1[25:6];
		cp0_entrylo0_flags <= alu_result1[5:0];
	end
	else if (pipe2_data[207]) begin//tlbr
		cp0_entrylo0_pfn   <= Entrylo0_out[25:6];
		cp0_entrylo0_flags <= Entrylo0_out[5:0];
	end
	
end

//entrylo1      reg:3
always @(posedge clk) begin
	if (rst) begin
		cp0_entrylo1_pfn    <= 20'd0;
		cp0_entrylo1_flags  <= 6'd0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd3) begin
		cp0_entrylo1_pfn   <= alu_result1[25:6];
		cp0_entrylo1_flags <= alu_result1[5:0];
	end
	else if (pipe2_data[207]) begin//tlbr
		cp0_entrylo1_pfn   <= Entrylo1_out[25:6];
		cp0_entrylo1_flags <= Entrylo1_out[5:0];
	end
	
end

//pagemask      reg:5
always @(posedge clk) begin
	if (rst) begin
		cp0_pagemask_mask    <= 12'd0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd5) begin
		cp0_pagemask_mask    <= alu_result1[24:13];
	end
	else if (pipe2_data[207]) begin
		cp0_pagemask_mask    <= mask;
	end
end

//entryhi     reg:10
always @(posedge clk) begin
	if (rst) begin
		cp0_entryhi_vpn2    <= 19'd0;
		cp0_entryhi_asid    <= 8'd0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd10) begin
		cp0_entryhi_vpn2    <= alu_result1[31:13];
		cp0_entryhi_asid    <= alu_result1[7:0];
	end
	else if (pipe2_data[207]) begin //tlbr
		cp0_entryhi_vpn2    <= Entryhi_out[31:13];
		cp0_entryhi_asid    <= Entryhi_out[7:0];
	end
	else if (~cp0_status_EXL & (pipe2_data[209] | pipe2_data[210])) begin//fetch tlb refill/invalid
		cp0_entryhi_vpn2    <= pipe2_data[151:133]; //pc
	end
	else if (~cp0_status_EXL &(
			 (tlb_refill_data == 1'b1 & pipe2_data[193:191] != 3'b101) | //sw
			 (tlb_refill_data == 1'b1 & pipe2_data[190:188] != 3'b111) | //lw
			 (tlb_invalid_data == 1'b1 & pipe2_data[193:191] != 3'b101) | //sw
			 (tlb_invalid_data == 1'b1 & pipe2_data[190:188] != 3'b111) |//lw
			 (tlb_modified == 1'b1 & pipe2_data[193:192] != 3'b101))
			 ) begin
		cp0_entryhi_vpn2	<= alu_result1[31:13];//load / store tlb refill/invalid
	end
	
end

wire debug_entryhi1;
assign debug_entryhi1 = pipe2_data[209] | pipe2_data[210];
wire debug_entryhi2;
assign debug_entryhi2 = ((tlb_refill_data == 1'b1 & pipe2_data[193:191] != 3'b101) | //sw
			 (tlb_refill_data == 1'b1 & pipe2_data[190:188] != 3'b111) | //lw
			 (tlb_invalid_data == 1'b1 & pipe2_data[193:191] != 3'b101) | //sw
			 (tlb_invalid_data == 1'b1 & pipe2_data[190:188] != 3'b111) |//lw
			 (tlb_modified == 1'b1 & pipe2_data[193:192] != 3'b101)
			 );


//badvaddr      reg:8
always @(posedge clk) begin
	if (rst) begin
		cp0_Badvaddr <= 32'd0;
		
	end
	else if (~cp0_status_EXL & (pipe2_data[206] | pipe2_data[209] | pipe2_data[210]) ) begin//fetch pc
		cp0_Badvaddr <= pipe2_data[151:120];
	end
	else if (~cp0_status_EXL & (
		((pipe2_data[190:188] == 3'd3) & (alu_result1[0] != 1'b0)) |//lh 
		((pipe2_data[190:188] == 3'd4) & (alu_result1[0] != 1'b0)) |//lhu
		((pipe2_data[190:188] == 3'd0) & (alu_result1[1:0] != 2'b00)) | //LW
		((pipe2_data[193:191] == 3'd2) & (alu_result1[0] != 1'd0)) |//sh
		((pipe2_data[193:191] == 3'd0) & (alu_result1[1:0] != 2'b00)) | //sw
		(tlb_refill_data == 1'b1 & pipe2_data[190:188] != 3'b111) |//load tlb refill
		(tlb_refill_data == 1'b1 & pipe2_data[193:191] != 3'b101) | //store tlb refill
		(tlb_invalid_data == 1'b1 & pipe2_data[190:188] != 3'b111)| //load tlb invalid
		(tlb_invalid_data == 1'b1 & pipe2_data[193:191] != 3'b101) | //store tlb invalid
		(tlb_modified == 1'b1 & pipe2_data[193:192] != 3'b101)//store tlb modified
		))begin
		cp0_Badvaddr <= alu_result1;
	end

end


/////count      reg:9
always @(posedge clk) begin
	if (rst) begin
		cp0_count <= 32'd0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd9) begin
		cp0_count <= alu_result1;
	end
	else if (count_add_en) begin
		cp0_count <= cp0_count + 1'b1;
	end
end

/////compare  reg:11
always @(posedge clk) begin
	if (rst) begin
		cp0_compare <= 32'h0;
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd11) begin
		cp0_compare <= alu_result1;
	end
end


/////status  reg:11
always @(posedge clk) begin
	if (rst) begin
		cp0_status_IM7 <= 1'b0;
		cp0_status_IM6 <= 1'b0;
		cp0_status_IM5 <= 1'b0;
		cp0_status_IM4 <= 1'b0;
		cp0_status_IM3 <= 1'b0;
		cp0_status_IM2 <= 1'b0;
		cp0_status_IM1 <= 1'b0;
		cp0_status_IM0 <= 1'b0;
		cp0_status_EXL <= 1'b0;
		cp0_status_IE  <= 1'b0;		
	end
	else begin
		//exl [1]
		if (ex_valid | tlb_refill ) begin
			cp0_status_EXL <= 1'b1;//SYSCALL break maintain_inst
		end
		else if (pipe2_data[202]) begin//ERET
			cp0_status_EXL <= 1'b0;
		end
		else if (pipe2_data[194] && pipe2_data[199:195] == 5'd12) begin
			cp0_status_EXL <= alu_result1[1];
		end

		//other
		if (pipe2_data[194] && pipe2_data[199:195] == 5'd12) begin
			cp0_status_IM7 <= alu_result1[15];
			cp0_status_IM6 <= alu_result1[14];
			cp0_status_IM5 <= alu_result1[13];
			cp0_status_IM4 <= alu_result1[12];
			cp0_status_IM3 <= alu_result1[11];
			cp0_status_IM2 <= alu_result1[10];
			cp0_status_IM1 <= alu_result1[9];
			cp0_status_IM0 <= alu_result1[8];
			cp0_status_IE  <= alu_result1[0];
		end
	end
end


				
//cause    reg:13
always @(posedge clk) begin
	if (rst) begin
		cp0_cause_TI <= 1'b0;
	end
	
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd11) begin//write compare
		cp0_cause_TI <= 1'b0;
	end
	else if (count_cmp_eq) begin
		cp0_cause_TI <= 1'b1;
	end
	
	if (rst) begin
		cp0_cause_BD      <= 1'b0; 
 		cp0_cause_IP7     <= 1'b0;
 		cp0_cause_IP6     <= 1'b0;
 		cp0_cause_IP5     <= 1'b0;
 		cp0_cause_IP4     <= 1'b0;
 		cp0_cause_IP3     <= 1'b0;
 		cp0_cause_IP2     <= 1'b0;
 		cp0_cause_IP1     <= 1'b0;
 		cp0_cause_IP0     <= 1'b0;
 		cp0_cause_ExcCode <= 5'h1f;
	end
	else begin
		if (ex_valid | tlb_refill) begin
			if (INT) begin//int
				cp0_cause_ExcCode <= 5'h0;
			end
			else if (pipe2_data[204]) begin
				cp0_cause_ExcCode <= 5'ha;//maintain
			end
			else if (pipe2_data[201]) begin
				cp0_cause_ExcCode <= 5'h8;//syscall
			end
			else if (pipe2_data[205] & overflow) begin
				cp0_cause_ExcCode <= 5'hc;//overflow
			end
			else if (pipe2_data[203]) begin
				cp0_cause_ExcCode <= 5'h9;//break
			end
			else if ((pipe2_data[190:188] == 3'd3 & alu_result1[0] != 1'b0) |//lh 
					(pipe2_data[190:188] == 3'd4 & alu_result1[0] != 1'b0) |//lhu
					(pipe2_data[190:188] == 3'd0 & alu_result1[1:0] != 2'd0) |
					pipe2_data[206]) 
			begin
				cp0_cause_ExcCode <= 5'h4;//lw error & fetch
			end
			else if ((pipe2_data[193:191] == 3'd2 & alu_result1[0] != 1'd0) |//sh
					(pipe2_data[193:191] == 3'd0 & alu_result1[1:0] != 2'd0)) begin
				cp0_cause_ExcCode <= 5'h5;//sw error
			end
			else if ((tlb_refill_data == 1'b1 & pipe2_data[193:191] != 3'b101) | (tlb_invalid_data == 1'b1 & pipe2_data[193:191] != 3'b101)) begin
				cp0_cause_ExcCode <= 5'h3;//tlb sw refill/invalid error
			end
			else if ((tlb_refill_data == 1'b1 & pipe2_data[190:188] != 3'b111) | pipe2_data[209] | (tlb_invalid_data == 1'b1 & pipe2_data[190:188] != 3'b111) | pipe2_data[210]) begin
				cp0_cause_ExcCode <= 5'h2;//tlb lw / fetch  refill/invalid error
			end
			else if (tlb_modified == 1'b1 & pipe2_data[193:192] != 3'b101) begin
				cp0_cause_ExcCode <= 5'h1;//tlb modified
			end
			

			if (pipe3_data[229]) begin
				cp0_cause_BD <= 1'b1;
			end
			else if (~pipe3_data[229]) begin
				cp0_cause_BD <= 1'b0;
			end
		end

		if (pipe2_data[194] && pipe2_data[199:195] == 5'd13) begin
			cp0_cause_IP1 <= alu_result1[9];
			cp0_cause_IP0 <= alu_result1[8];
		end

		cp0_cause_IP7 <= int_pending[5];
		cp0_cause_IP6 <= int_pending[4];
		cp0_cause_IP5 <= int_pending[3];
		cp0_cause_IP4 <= int_pending[2];
		cp0_cause_IP3 <= int_pending[1];
		cp0_cause_IP2 <= int_pending[0];
	end
end

assign Need_int = (cp0_cause_ExcCode != 5'h1f)?1:0;

always @(posedge clk) begin
	if (rst) begin
		Need_int2 <= 1'b0;
	end
	else begin
		Need_int2 <= Need_int;
	end
end

/////epc  reg:14
///////////writed in exe_stage
always @(posedge clk) begin
	if (rst) begin
		cp0_epc <= 32'd0;
		
	end
	else if (pipe2_data[194] && pipe2_data[199:195] == 5'd14) begin
		cp0_epc <= alu_result1;//mtc0
	end
	else if (pipe3_data[229] && (ex_valid | tlb_refill) ) begin //branch && ~cp0_status_EXL
		cp0_epc <= pipe3_data[151:120]; //PC-4 (j pc)
	end
	else if (~pipe3_data[229] && (ex_valid | tlb_refill) ) begin //&& ~cp0_status_EXL
		cp0_epc <= pipe2_data[151:120]; //current PC
	end
end



/*----------------------------pipe3----------------------------*/
 assign pipe3_ready_go       = pipe3_valid;
 assign pipe3_allowin        = !pipe3_valid || (pipe3_ready_go && pipe4_allowin);
 assign pipe3_to_pipe4_valid = pipe3_valid && pipe3_ready_go;
 
 always @(posedge clk) begin
     if(rst) begin
	     pipe3_valid <= 1'b0;
	   end
	 else if(pipe3_allowin) begin
	     pipe3_valid <= pipe2_to_pipe3_valid;
	   end
	 if(pipe2_to_pipe3_valid && pipe3_allowin) begin
		 pipe3_data[234]     <= pipe2_data[208];//TLBP
	 	 pipe3_data[233]     <= pipe2_data[207];//TLBR
	 	 pipe3_data[232]     <= ex_valid | tlb_refill;
	 	 pipe3_data[231]     <= pipe2_data[202];//eret
	 	 pipe3_data[230]     <= pipe2_data[201];     // SYSCALL;
         pipe3_data[229]     <= pipe2_data[200];     //branch
	 	 pipe3_data[228:224] <= pipe2_data[199:195]; //cp0_waddr
	 	 pipe3_data[223] 	 <= pipe2_data[194];	 //cp0_wen
	 	 pipe3_data[222:220] <= pipe2_data[190:188];//load_type
	     pipe3_data[219:216] <= pipe2_data[187:184];// 0
	 	 pipe3_data[215:184] <= wdata; //wdata from alu
	     pipe3_data[183:152] <= pipe2_data[183:152];//rdata2 for SW
	     pipe3_data[151:120] <= pipe2_data[151:120];//PC
	     pipe3_data[119: 88] <= pipe2_data[119:88]; //PC+8
		 pipe3_data[ 87: 56] <= alu_result1;         //Address
		 pipe3_data[ 55: 24] <= pipe2_data[55:24];  //data2, probably data_ram write data
		 pipe3_data[ 23: 23] <= pipe2_data[19:19];  //data_sram_en
		 pipe3_data[ 22: 22] <= pipe2_data[18:18] & ~ex_valid & ~tlb_refill;  //Mem_wen
		 pipe3_data[ 21: 21] <= pipe2_data[17:17] & ~ex_valid & ~tlb_refill;//RegWrite
		 pipe3_data[ 20: 16] <= pipe2_data[16:12];  //waddr
		 pipe3_data[ 15: 13] <= pipe2_data[11:9];   //MemtoReg
		 pipe3_data[ 12: 12] <= carryout;
		 pipe3_data[ 11:  8] <= pipe2_data[8:5]; //DIV, DIVU, MULT, MULTU
		 pipe3_data[  7:  6] <= pipe2_data[4:3]; //MTHI MTLO
		 pipe3_data[  5:  4] <= alu_result1[1:0]; //offset[1:0]
		 pipe3_data[  3:  0] <= 4'b0;
	   end
 end

assign mem_pc = pipe3_data[151:120];
wire debug_mem_Mem;
wire debug_mem_RegWrite;

assign debug_mem_Mem = pipe3_data[22];
assign debug_mem_RegWrite = pipe3_data[21];

/*
assign wdata=({32{pipe3_data[15:13]==3'd0}} & pipe3_data[87:56])//ALUresult1
           | ({32{pipe3_data[15:13]==3'd1}} & Read_data)
		   | ({32{pipe3_data[15:13]==3'd2}} & pipe3_data[55:24])
		   | ({32{pipe3_data[15:13]==3'd3}} & pipe3_data[12:12])//slt(Carryout)
		   | ({32{pipe3_data[15:13]==3'd4}} & pipe3_data[119:88]);//PC+8
*/
wire [31: 0] wdata_final;
wire [ 7: 0] Read_data_8;
wire [15: 0] Read_data_16;
wire [ 4: 0] bitnumber;
wire [31: 0] LWL_wdata;
wire [31: 0] LWR_wdata;

assign bitnumber    = 	pipe3_data[5:4] << 3;  
assign Read_data_8  = 	(pipe3_data[5:4] == 2'b00)?Read_data[7:0]:
						(pipe3_data[5:4] == 2'b01)?Read_data[15:8]:
						(pipe3_data[5:4] == 2'b10)?Read_data[23:16]:
						(pipe3_data[5:4] == 2'b11)?Read_data[31:24]:
						8'b0; //(pipe3_data[5:4] == 2'b11)

assign Read_data_16 = 	(pipe3_data[5:4] == 2'b00)?Read_data[15:0]:
						(pipe3_data[5:4] == 2'b10)?Read_data[31:16]:
						16'b0;  //Read_data[(bitnumber+15):bitnumber];

assign LWL_wdata    =	(pipe3_data[5:4] == 2'b00)?{Read_data[7:0],pipe3_data[175:152]}://[183:152]
						(pipe3_data[5:4] == 2'b01)?{Read_data[15:0],pipe3_data[167:152]}:
						(pipe3_data[5:4] == 2'b10)?{Read_data[23:0],pipe3_data[159:152]}:
						Read_data;

assign LWR_wdata    =	(pipe3_data[5:4] == 2'b00)?Read_data:
						(pipe3_data[5:4] == 2'b01)?{pipe3_data[183:176], Read_data[31:8]}://[183:152]
						(pipe3_data[5:4] == 2'b10)?{pipe3_data[183:168], Read_data[31:16]}:
						{pipe3_data[183:160], Read_data[31:24]};

assign wdata_final  =   (pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b000)?Read_data:// LW
						(pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b001)?{{24{Read_data_8[7]}}, Read_data_8}:  //LB
						(pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b010)?{24'b0, Read_data_8}: //LBU
						(pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b011)?{{16{Read_data_16[15]}}, Read_data_16}:  //LH
						(pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b100)?{16'b0, Read_data_16}: //LHU
						(pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b101)?LWL_wdata:  //LWL 
						(pipe3_data[15:13] == 3'd1 && pipe3_data[222:220] == 3'b110)?LWR_wdata:  //LWR
						pipe3_data[215:184];
 
/*----------------------------pipe4----------------------------*/
 //wire valid_out;
 //wire [WIDTH-1:0] data_out;
 assign pipe4_ready_go=pipe4_valid;
 assign pipe4_allowin=!pipe4_valid || pipe4_ready_go /*&& out_allow*/;
 always @(posedge clk)
 begin
     if(rst) begin
	     pipe4_valid <= 1'b0;
	   end
	 else if(pipe4_allowin) begin
	     pipe4_valid <= pipe3_to_pipe4_valid;
	   end
	 if(pipe3_to_pipe4_valid && pipe4_allowin) begin
	 	 pipe4_data[152]     <= pipe3_data[232];//ex_valid | tlb_refill
	     pipe4_data[151:120] <= pipe3_data[151:120];//PC
	     pipe4_data[119: 88] <= wdata_final;              //wdata
		 pipe4_data[ 87: 83] <= pipe3_data[ 20: 16];//waddr
		 pipe4_data[ 82: 82] <= pipe3_data[ 21: 21];//RegWrite
		 pipe4_data[ 81: 79] <= pipe3_data[ 15: 13];//memtoreg to memorize if is lw
		 pipe4_data[ 78: 75] <= pipe3_data[ 11:  8]; ////DIV, DIVU, MULT, MULTU
		 pipe4_data[ 74: 73] <= pipe3_data[  7:  6]; // MTHI MTLO
	 	 pipe4_data[ 72: 68] <= pipe3_data[228:224]; //cp0_waddr
	 	 pipe4_data[67]      <= pipe3_data[223];	 //cp0_wen
		 pipe4_data[ 66:  0] <= 0;
	   end 
 end 
 //assign valid_out=pipe4_valid && pipe4_ready_go;
 //assign data_out=pipe4_data;


//debug
assign wb_pc = pipe4_data[151:120];

//Interface
assign rst               = ~resetn;

always@(posedge clk) begin
    rst2 <= resetn;
end
always@(posedge clk) begin
    rst3 <= rst2;
end
always@(posedge clk) begin
    rst4 <= rst3;
end
always@(posedge clk) begin
    rst5 <= rst4;
end



assign inst_sram_addr    = (inst_vaddr < 32'h80000000)?inst_paddr:inst_vaddr;
assign inst_vaddr        = (rst)?PC:
							(pipe1_ready_go==0)?PC:PC_next;//PC_next;
assign data_sram_addr    = (data_vaddr < 32'h80000000)?data_paddr:data_vaddr;
assign data_vaddr 		 = Address;
assign data_sram_wdata   = Write_data;
assign Read_data         = data_sram_rdata;
assign debug_wb_rf_wnum  = pipe4_data[87:83];
assign debug_wb_rf_wdata = pipe4_data[119:88];
assign debug_wb_pc       = pipe4_data[151:120];
//assign debug_wb_rf_wen={4{pipe4_data[82:82]}};
assign debug_wb_rf_wen={4{wb_RegWrite}};	





//LO HI
always @(posedge clk) begin
	if (rst) begin
		HI <= 32'b0;		
	end
	else if (pipe4_data[74]) begin
		HI <= pipe4_data[119:88];   //MTHI
	end
	else if (pipe4_data[76] || pipe4_data[75]) begin
	    HI <= mul_result[63:32];  //MUL
	end
	else if ((pipe4_data[78] || pipe4_data[77]) && m_axis_dout_tvalid) begin
		HI <= m_axis_dout_tdata[31:0];  //DIV
	end
end
/*
wire [31:0] debug_mul_result;
wire [31:0] debug_div_result;
wire [31:0] debug_write_LO;
wire [3:0] signal;
assign signal = pipe4_data[76:73];
assign debug_mul_result = mul_result[31:0];
assign debug_div_result = m_axis_dout_tdata[71:40];
assign debug_write_LO = pipe4_data[119:88];
*/
always @(posedge clk) begin
	if (rst) begin
		LO <= 32'b0;
		
	end
	else if (pipe4_data[73]) begin
		LO <= pipe4_data[119:88]; //MTLO
	end
	else if (pipe4_data[76] || pipe4_data[75]) begin
	    LO <= mul_result[31:0];  //MUL
	end
	else if ((pipe4_data[78] || pipe4_data[77]) && m_axis_dout_tvalid) begin
		LO <= m_axis_dout_tdata[71:40];  //DIV
	end
end
//-----------------------------------------------------------------------------------------------------------

//regfile
/*regfile r(
	.clk(clk),
	.rst(rst),
	.waddr(pipe4_data[87:83]),
	.raddr1(pipe1_data[25:21]),
	.raddr2(pipe1_data[20:16]),
	.wen(pipe4_data[82:82]),
	.wdata(pipe4_data[119:88]),
	.rdata1(rdata1),
	.rdata2(rdata2)
);*/
regfile r(
	.clk(clk),
	.rst(rst),
	.waddr(pipe4_data[87:83]),
	.raddr1(pipe1_data[25:21]),
	.raddr2(pipe1_data[20:16]),
	.wen(wb_RegWrite),
	.wdata(pipe4_data[119:88]),
	.rdata1(rdata1),
	.rdata2(rdata2)
);


//ALU

//execute: alu
alu a1(
	.A(pipe2_data[87:56]),
	.B(pipe2_data[55:24]),
	.ALUop(pipe2_data[23:20]),
	.Overflow(overflow),
	.CarryOut(carryout),
	.Zero(),
    .Result(alu_result1)
);

//PC+offset: alu
alu a2(
	.A(PC),
	.B(offset),
	.ALUop(4'b0010),
	.Overflow(),
	.CarryOut(),
	.Zero(),
    .Result(alu_result2)
);

//sign extension: alu
alu a3(
	.A(rdata1_true),
	.B(rdata2_true),
	.ALUop(4'b0011),
	.Overflow(),
	.CarryOut(),
	.Zero(zero),
    .Result()
);
//sent out at 3 stage, comback at 5 
assign mul_a = ({33{pipe2_data[6]}} & {pipe2_data[87],pipe2_data[87:56]}) | ({33{pipe2_data[5]}} & {1'b0,pipe2_data[87:56]});
assign mul_b = ({33{pipe2_data[6]}} & {pipe2_data[55],pipe2_data[55:24]}) | ({33{pipe2_data[5]}} & {1'b0,pipe2_data[55:24]});

mymul mul1(
     .CLK(clk),
     .A(mul_a),
     .B(mul_b),
     .P(mul_result)  
);

always @(posedge clk) begin
	if (rst) begin
		s_axis_divisor_tvalid <= 1'b0;
	end
	else if (DIV || DIVU) begin
        s_axis_divisor_tvalid <= 1'b1;
    end
	else if (s_axis_divisor_tready) begin
		s_axis_divisor_tvalid <= 1'b0;
	end
end

always @(posedge clk) begin
	if (rst) begin
		s_axis_dividend_tvalid <= 1'b0;
	end
	else if (DIV || DIVU) begin
        s_axis_dividend_tvalid <= 1'b1;
    end
	else if (s_axis_divisor_tready) begin
		s_axis_dividend_tvalid <= 1'b0;
	end

end

//assign s_axis_divisor_tdata = ({40{DIV}} & {{8{data2[31]}},data2[31:0]}) | ({40{DIVU}} & {8'b0,data2[31:0]});
//assign s_axis_dividend_tdata = ({40{DIV}} & {{8{data1[31]}},data1[31:0]}) | ({40{DIVU}} & {8'b0,data1[31:0]});


assign s_axis_divisor_tdata = ({40{pipe2_data[8]}} & {{8{pipe2_data[55]}},pipe2_data[55:24]}) | ({40{pipe2_data[7]}} & {8'b0,pipe2_data[55:24]});
assign s_axis_dividend_tdata = ({40{pipe2_data[8]}} & {{8{pipe2_data[87]}},pipe2_data[87:56]}) | ({40{pipe2_data[7]}} & {8'b0,pipe2_data[87:56]});

mydiv div1(
     .s_axis_divisor_tdata    (s_axis_divisor_tdata  ),
     .s_axis_divisor_tready   (s_axis_divisor_tready ),
     .s_axis_divisor_tvalid   (s_axis_divisor_tvalid ),
     .s_axis_dividend_tdata   (s_axis_dividend_tdata ),
     .s_axis_dividend_tready  (s_axis_dividend_tready),
     .s_axis_dividend_tvalid  (s_axis_dividend_tvalid),
     .aclk                    (clk                   ),
     .m_axis_dout_tdata       (m_axis_dout_tdata     ),
     .m_axis_dout_tvalid      (m_axis_dout_tvalid    )
);


always @(posedge clk) begin
	if (rst) begin
		tlb_refill_inst_reg <= 1'b0;
	end
	else begin
		tlb_refill_inst_reg <= tlb_refill_inst;
	end

end

always @(posedge clk) begin
	if (rst) begin
		tlb_invalid_inst_reg <= 1'b0;
	end
	else begin
		tlb_invalid_inst_reg <= tlb_invalid_inst;
	end

end


mytlb tlb1(
	.clk 						(clk 				),
	.index 						(cp0_index_true[4:0]),
	.Entryhi 					(cp0_entryhi_true 	),
	.Pagemask 					(cp0_pagemask_true 	),
	.Entrylo0 					(cp0_entrylo0_true 	),
	.Entrylo1 					(cp0_entrylo1_true 	),
	.vaddr_inst 				(inst_vaddr			),
	.vaddr_data  				(data_vaddr			),
	.TLBWI						(TLBWI 				),
	.TLBR						(TLBR 				),
	.TLBP 						(TLBP 				),

	.index_find 				(index_find 		),
	.index_out					(index_out 			),
	.Entryhi_out				(Entryhi_out 		),
	.mask 						(mask 				),
	.Entrylo0_out 				(Entrylo0_out		),
	.Entrylo1_out 				(Entrylo1_out 		),
	.paddr_inst 				(inst_paddr 		),
	.paddr_data					(data_paddr 		),
	.tlb_refill_inst			(tlb_refill_inst 	),
	.tlb_refill_data			(tlb_refill_data 	),
	.tlb_invalid_inst			(tlb_invalid_inst	),
	.tlb_invalid_data			(tlb_invalid_data	),
	.tlb_modified				(tlb_modified		)

);





endmodule
