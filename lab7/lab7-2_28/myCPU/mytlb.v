module mytlb(
	input 		  clk,

	input  [ 4:0] index,
	input  [31:0] Entryhi,
	input  [31:0] Pagemask,
	input  [31:0] Entrylo0,
	input  [31:0] Entrylo1,
	input  [31:0] vaddr_inst,
	input  [31:0] vaddr_data,
	input	      TLBWI,
	input         TLBR,
	input         TLBP,

	output reg        index_find,
	output reg [ 4:0] index_out,
	output reg [31:0] Entryhi_out,
	output reg [11:0] mask,
	output reg [25:0] Entrylo0_out,
	output reg [25:0] Entrylo1_out,
	output 	   [31:0] paddr_inst,
	output 	   [31:0] paddr_data
	);

reg [18:0] VPN2 [31:0];
reg [ 7:0] ASID [31:0];
reg [11:0] MASK [31:0];

reg 	   G 	[31:0];
//phy0
reg [19:0] PFN0 [31:0];
reg [ 4:0] CDV0 [31:0];
//phy1
reg [19:0] PFN1 [31:0];
reg [ 4:0] CDV1 [31:0];

wire [19:0] PFN_inst [31:0];
wire [19:0] PFN_data [31:0];

//if vaddr[31:13] == vpn2[i]
wire [31:0]	hit_inst;
wire [31:0] hit_data;
wire [31:0] tlbp_hit;



assign tlbp_hit[0] = ((Entryhi[31:13] == VPN2[0]) && (G[0] || (Entryhi[7:0] == ASID[0])))?1:0;
assign tlbp_hit[1] = ((Entryhi[31:13] == VPN2[1]) && (G[1] || (Entryhi[7:0] == ASID[1])))?1:0;
assign tlbp_hit[2] = ((Entryhi[31:13] == VPN2[2]) && (G[2] || (Entryhi[7:0] == ASID[2])))?1:0;
assign tlbp_hit[3] = ((Entryhi[31:13] == VPN2[3]) && (G[3] || (Entryhi[7:0] == ASID[3])))?1:0;
assign tlbp_hit[4] = ((Entryhi[31:13] == VPN2[4]) && (G[4] || (Entryhi[7:0] == ASID[4])))?1:0;
assign tlbp_hit[5] = ((Entryhi[31:13] == VPN2[5]) && (G[5] || (Entryhi[7:0] == ASID[5])))?1:0;
assign tlbp_hit[6] = ((Entryhi[31:13] == VPN2[6]) && (G[6] || (Entryhi[7:0] == ASID[6])))?1:0;
assign tlbp_hit[7] = ((Entryhi[31:13] == VPN2[7]) && (G[7] || (Entryhi[7:0] == ASID[7])))?1:0;
assign tlbp_hit[8] = ((Entryhi[31:13] == VPN2[8]) && (G[8] || (Entryhi[7:0] == ASID[8])))?1:0;
assign tlbp_hit[9] = ((Entryhi[31:13] == VPN2[9]) && (G[9] || (Entryhi[7:0] == ASID[9])))?1:0;

assign tlbp_hit[10] = ((Entryhi[31:13] == VPN2[10]) && (G[10] || (Entryhi[7:0] == ASID[10])))?1:0;
assign tlbp_hit[11] = ((Entryhi[31:13] == VPN2[11]) && (G[11] || (Entryhi[7:0] == ASID[11])))?1:0;
assign tlbp_hit[12] = ((Entryhi[31:13] == VPN2[12]) && (G[12] || (Entryhi[7:0] == ASID[12])))?1:0;
assign tlbp_hit[13] = ((Entryhi[31:13] == VPN2[13]) && (G[13] || (Entryhi[7:0] == ASID[13])))?1:0;
assign tlbp_hit[14] = ((Entryhi[31:13] == VPN2[14]) && (G[14] || (Entryhi[7:0] == ASID[14])))?1:0;
assign tlbp_hit[15] = ((Entryhi[31:13] == VPN2[15]) && (G[15] || (Entryhi[7:0] == ASID[15])))?1:0;
assign tlbp_hit[16] = ((Entryhi[31:13] == VPN2[16]) && (G[16] || (Entryhi[7:0] == ASID[16])))?1:0;
assign tlbp_hit[17] = ((Entryhi[31:13] == VPN2[17]) && (G[17] || (Entryhi[7:0] == ASID[17])))?1:0;
assign tlbp_hit[18] = ((Entryhi[31:13] == VPN2[18]) && (G[18] || (Entryhi[7:0] == ASID[18])))?1:0;
assign tlbp_hit[19] = ((Entryhi[31:13] == VPN2[19]) && (G[19] || (Entryhi[7:0] == ASID[19])))?1:0;
assign tlbp_hit[20] = ((Entryhi[31:13] == VPN2[20]) && (G[20] || (Entryhi[7:0] == ASID[20])))?1:0;
assign tlbp_hit[21] = ((Entryhi[31:13] == VPN2[21]) && (G[21] || (Entryhi[7:0] == ASID[21])))?1:0;
assign tlbp_hit[22] = ((Entryhi[31:13] == VPN2[22]) && (G[22] || (Entryhi[7:0] == ASID[22])))?1:0;
assign tlbp_hit[23] = ((Entryhi[31:13] == VPN2[23]) && (G[23] || (Entryhi[7:0] == ASID[23])))?1:0;
assign tlbp_hit[24] = ((Entryhi[31:13] == VPN2[24]) && (G[24] || (Entryhi[7:0] == ASID[24])))?1:0;
assign tlbp_hit[25] = ((Entryhi[31:13] == VPN2[25]) && (G[25] || (Entryhi[7:0] == ASID[25])))?1:0;
assign tlbp_hit[26] = ((Entryhi[31:13] == VPN2[26]) && (G[26] || (Entryhi[7:0] == ASID[26])))?1:0;
assign tlbp_hit[27] = ((Entryhi[31:13] == VPN2[27]) && (G[27] || (Entryhi[7:0] == ASID[27])))?1:0;
assign tlbp_hit[28] = ((Entryhi[31:13] == VPN2[28]) && (G[28] || (Entryhi[7:0] == ASID[28])))?1:0;
assign tlbp_hit[29] = ((Entryhi[31:13] == VPN2[29]) && (G[29] || (Entryhi[7:0] == ASID[29])))?1:0;
assign tlbp_hit[30] = ((Entryhi[31:13] == VPN2[30]) && (G[30] || (Entryhi[7:0] == ASID[30])))?1:0;
assign tlbp_hit[31] = ((Entryhi[31:13] == VPN2[31]) && (G[31] || (Entryhi[7:0] == ASID[31])))?1:0;



// tlbwi tlbr tlbp
always @(posedge clk) begin
	if (TLBWI) begin
		VPN2[index] <= Entryhi[31:13];
		ASID[index] <= Entryhi[7:0];
		MASK[index] <= Pagemask[24:13];
		G[index]	<= Entrylo0[0] & Entrylo1[0];
		PFN0[index] <= Entrylo0[25:6];
		CDV0[index] <= Entrylo0[5:1];
		PFN1[index] <= Entrylo1[25:6];
		CDV1[index] <= Entrylo1[5:1];
	end
	else if (TLBR) begin
		Entryhi_out  <= {VPN2[index], 5'd0, ASID[index]};
		mask     <= MASK[index];
		Entrylo0_out <= {PFN0[index], CDV0[index], G[index]};
		Entrylo1_out <= {PFN1[index], CDV1[index], G[index]};
		
	end
	else if (TLBP) begin
		if (tlbp_hit[31:0] != 32'd0) begin
			index_out <= 	({5{tlbp_hit[0]}}& 5'd0 )  | ({5{tlbp_hit[1]}}& 5'd1 )|
							({5{tlbp_hit[2]}}& 5'd2 )  | ({5{tlbp_hit[3]}}& 5'd3 )|
							({5{tlbp_hit[4]}}& 5'd4 )  | ({5{tlbp_hit[5]}}& 5'd5 )|
							({5{tlbp_hit[6]}}& 5'd6 )  | ({5{tlbp_hit[7]}}& 5'd7 )|
							({5{tlbp_hit[8]}}& 5'd8 )  | ({5{tlbp_hit[9]}}& 5'd9 )|
							({5{tlbp_hit[10]}} & 5'd10) | ({5{tlbp_hit[11]}} & 5'd11) |
							({5{tlbp_hit[12]}} & 5'd12) | ({5{tlbp_hit[13]}} & 5'd13) |
							({5{tlbp_hit[14]}} & 5'd14) | ({5{tlbp_hit[15]}} & 5'd15) |
							({5{tlbp_hit[16]}} & 5'd16) | ({5{tlbp_hit[17]}} & 5'd17) |
							({5{tlbp_hit[18]}} & 5'd18) | ({5{tlbp_hit[19]}} & 5'd19) |
							({5{tlbp_hit[20]}} & 5'd20) | ({5{tlbp_hit[21]}} & 5'd21) |
							({5{tlbp_hit[22]}} & 5'd22) | ({5{tlbp_hit[23]}} & 5'd23) |
							({5{tlbp_hit[24]}} & 5'd24) | ({5{tlbp_hit[25]}} & 5'd25) |
							({5{tlbp_hit[26]}} & 5'd26) | ({5{tlbp_hit[27]}} & 5'd27) |
							({5{tlbp_hit[28]}} & 5'd28) | ({5{tlbp_hit[29]}} & 5'd29) |
							({5{tlbp_hit[30]}} & 5'd30) | ({5{tlbp_hit[31]}} & 5'd31) ;
			index_find <=   1'd0;
		end
		else begin
			index_out  <= 5'd0;
			index_find <= 1'd1;
		end
	end
end









/*look up for paddr_inst*/
assign hit_inst[0] = (vaddr_inst[31:13] == VPN2[0] && (G[0] || (Entryhi[7:0] == ASID[0])))?1:0;
assign hit_inst[1] = (vaddr_inst[31:13] == VPN2[1] && (G[1] || (Entryhi[7:0] == ASID[1])))?1:0;
assign hit_inst[2] = (vaddr_inst[31:13] == VPN2[2] && (G[2] || (Entryhi[7:0] == ASID[2])))?1:0;
assign hit_inst[3] = (vaddr_inst[31:13] == VPN2[3] && (G[3] || (Entryhi[7:0] == ASID[3])))?1:0;
assign hit_inst[4] = (vaddr_inst[31:13] == VPN2[4] && (G[4] || (Entryhi[7:0] == ASID[4])))?1:0;
assign hit_inst[5] = (vaddr_inst[31:13] == VPN2[5] && (G[5] || (Entryhi[7:0] == ASID[5])))?1:0;
assign hit_inst[6] = (vaddr_inst[31:13] == VPN2[6] && (G[6] || (Entryhi[7:0] == ASID[6])))?1:0;
assign hit_inst[7] = (vaddr_inst[31:13] == VPN2[7] && (G[7] || (Entryhi[7:0] == ASID[7])))?1:0;
assign hit_inst[8] = (vaddr_inst[31:13] == VPN2[8] && (G[8] || (Entryhi[7:0] == ASID[8])))?1:0;
assign hit_inst[9] = (vaddr_inst[31:13] == VPN2[9] && (G[9] || (Entryhi[7:0] == ASID[9])))?1:0;
assign hit_inst[10] = (vaddr_inst[31:13] == VPN2[10] && (G[10] || (Entryhi[7:0] == ASID[10])))?1:0;
assign hit_inst[11] = (vaddr_inst[31:13] == VPN2[11] && (G[11] || (Entryhi[7:0] == ASID[11])))?1:0;
assign hit_inst[12] = (vaddr_inst[31:13] == VPN2[12] && (G[12] || (Entryhi[7:0] == ASID[12])))?1:0;
assign hit_inst[13] = (vaddr_inst[31:13] == VPN2[13] && (G[13] || (Entryhi[7:0] == ASID[13])))?1:0;
assign hit_inst[14] = (vaddr_inst[31:13] == VPN2[14] && (G[14] || (Entryhi[7:0] == ASID[14])))?1:0;
assign hit_inst[15] = (vaddr_inst[31:13] == VPN2[15] && (G[15] || (Entryhi[7:0] == ASID[15])))?1:0;
assign hit_inst[16] = (vaddr_inst[31:13] == VPN2[16] && (G[16] || (Entryhi[7:0] == ASID[16])))?1:0;
assign hit_inst[17] = (vaddr_inst[31:13] == VPN2[17] && (G[17] || (Entryhi[7:0] == ASID[17])))?1:0;
assign hit_inst[18] = (vaddr_inst[31:13] == VPN2[18] && (G[18] || (Entryhi[7:0] == ASID[18])))?1:0;
assign hit_inst[19] = (vaddr_inst[31:13] == VPN2[19] && (G[19] || (Entryhi[7:0] == ASID[19])))?1:0;
assign hit_inst[20] = (vaddr_inst[31:13] == VPN2[20] && (G[20] || (Entryhi[7:0] == ASID[20])))?1:0;
assign hit_inst[21] = (vaddr_inst[31:13] == VPN2[21] && (G[21] || (Entryhi[7:0] == ASID[21])))?1:0;
assign hit_inst[22] = (vaddr_inst[31:13] == VPN2[22] && (G[22] || (Entryhi[7:0] == ASID[22])))?1:0;
assign hit_inst[23] = (vaddr_inst[31:13] == VPN2[23] && (G[23] || (Entryhi[7:0] == ASID[23])))?1:0;
assign hit_inst[24] = (vaddr_inst[31:13] == VPN2[24] && (G[24] || (Entryhi[7:0] == ASID[24])))?1:0;
assign hit_inst[25] = (vaddr_inst[31:13] == VPN2[25] && (G[25] || (Entryhi[7:0] == ASID[25])))?1:0;
assign hit_inst[26] = (vaddr_inst[31:13] == VPN2[26] && (G[26] || (Entryhi[7:0] == ASID[26])))?1:0;
assign hit_inst[27] = (vaddr_inst[31:13] == VPN2[27] && (G[27] || (Entryhi[7:0] == ASID[27])))?1:0;
assign hit_inst[28] = (vaddr_inst[31:13] == VPN2[28] && (G[28] || (Entryhi[7:0] == ASID[28])))?1:0;
assign hit_inst[29] = (vaddr_inst[31:13] == VPN2[29] && (G[29] || (Entryhi[7:0] == ASID[29])))?1:0;
assign hit_inst[30] = (vaddr_inst[31:13] == VPN2[30] && (G[30] || (Entryhi[7:0] == ASID[30])))?1:0;
assign hit_inst[31] = (vaddr_inst[31:13] == VPN2[31] && (G[31] || (Entryhi[7:0] == ASID[31])))?1:0;

assign PFN_inst[0] = (vaddr_inst[12])?PFN1[0]:PFN0[0];
assign PFN_inst[1] = (vaddr_inst[12])?PFN1[1]:PFN0[1];
assign PFN_inst[2] = (vaddr_inst[12])?PFN1[2]:PFN0[2];
assign PFN_inst[3] = (vaddr_inst[12])?PFN1[3]:PFN0[3];
assign PFN_inst[4] = (vaddr_inst[12])?PFN1[4]:PFN0[4];
assign PFN_inst[5] = (vaddr_inst[12])?PFN1[5]:PFN0[5];
assign PFN_inst[6] = (vaddr_inst[12])?PFN1[6]:PFN0[6];
assign PFN_inst[7] = (vaddr_inst[12])?PFN1[7]:PFN0[7];
assign PFN_inst[8] = (vaddr_inst[12])?PFN1[8]:PFN0[8];
assign PFN_inst[9] = (vaddr_inst[12])?PFN1[9]:PFN0[9];
assign PFN_inst[10] = (vaddr_inst[12])?PFN1[10]:PFN0[10];
assign PFN_inst[11] = (vaddr_inst[12])?PFN1[11]:PFN0[11];
assign PFN_inst[12] = (vaddr_inst[12])?PFN1[12]:PFN0[12];
assign PFN_inst[13] = (vaddr_inst[12])?PFN1[13]:PFN0[13];
assign PFN_inst[14] = (vaddr_inst[12])?PFN1[14]:PFN0[14];
assign PFN_inst[15] = (vaddr_inst[12])?PFN1[15]:PFN0[15];
assign PFN_inst[16] = (vaddr_inst[12])?PFN1[16]:PFN0[16];
assign PFN_inst[17] = (vaddr_inst[12])?PFN1[17]:PFN0[17];
assign PFN_inst[18] = (vaddr_inst[12])?PFN1[18]:PFN0[18];
assign PFN_inst[19] = (vaddr_inst[12])?PFN1[19]:PFN0[19];
assign PFN_inst[20] = (vaddr_inst[12])?PFN1[20]:PFN0[20];
assign PFN_inst[21] = (vaddr_inst[12])?PFN1[21]:PFN0[21];
assign PFN_inst[22] = (vaddr_inst[12])?PFN1[22]:PFN0[22];
assign PFN_inst[23] = (vaddr_inst[12])?PFN1[23]:PFN0[23];
assign PFN_inst[24] = (vaddr_inst[12])?PFN1[24]:PFN0[24];
assign PFN_inst[25] = (vaddr_inst[12])?PFN1[25]:PFN0[25];
assign PFN_inst[26] = (vaddr_inst[12])?PFN1[26]:PFN0[26];
assign PFN_inst[27] = (vaddr_inst[12])?PFN1[27]:PFN0[27];
assign PFN_inst[28] = (vaddr_inst[12])?PFN1[28]:PFN0[28];
assign PFN_inst[29] = (vaddr_inst[12])?PFN1[29]:PFN0[29];
assign PFN_inst[30] = (vaddr_inst[12])?PFN1[30]:PFN0[30];
assign PFN_inst[31] = (vaddr_inst[12])?PFN1[31]:PFN0[31];

assign paddr_inst[31:12] = 	({20{hit_inst[0]}}& PFN_inst[0])   | ({20{hit_inst[1]}}& PFN_inst[1]) |
							({20{hit_inst[2]}}& PFN_inst[2])   | ({20{hit_inst[3]}}& PFN_inst[3]) |
							({20{hit_inst[4]}}& PFN_inst[4])   | ({20{hit_inst[5]}}& PFN_inst[5]) |
							({20{hit_inst[6]}}& PFN_inst[6])   | ({20{hit_inst[7]}}& PFN_inst[7]) |
							({20{hit_inst[8]}}& PFN_inst[8])   | ({20{hit_inst[9]}}& PFN_inst[9]) |
							({20{hit_inst[10]}} & PFN_inst[10]) | ({20{hit_inst[11]}} & PFN_inst[11]) |
							({20{hit_inst[12]}} & PFN_inst[12]) | ({20{hit_inst[13]}} & PFN_inst[13]) |
							({20{hit_inst[14]}} & PFN_inst[14]) | ({20{hit_inst[15]}} & PFN_inst[15]) |
							({20{hit_inst[16]}} & PFN_inst[16]) | ({20{hit_inst[17]}} & PFN_inst[17]) |
							({20{hit_inst[18]}} & PFN_inst[18]) | ({20{hit_inst[19]}} & PFN_inst[19]) |
							({20{hit_inst[20]}} & PFN_inst[20]) | ({20{hit_inst[21]}} & PFN_inst[21]) |
							({20{hit_inst[22]}} & PFN_inst[22]) | ({20{hit_inst[23]}} & PFN_inst[23]) |
							({20{hit_inst[24]}} & PFN_inst[24]) | ({20{hit_inst[25]}} & PFN_inst[25]) |
							({20{hit_inst[26]}} & PFN_inst[26]) | ({20{hit_inst[27]}} & PFN_inst[27]) |
							({20{hit_inst[28]}} & PFN_inst[28]) | ({20{hit_inst[29]}} & PFN_inst[29]) |
							({20{hit_inst[30]}} & PFN_inst[30]) | ({20{hit_inst[31]}} & PFN_inst[31]) ;
							
assign paddr_inst[11: 0] = vaddr_inst[11:0];

/*look up for paddr_data*/
assign hit_data[0] = (vaddr_data[31:13] == VPN2[0] && (G[0] || (Entryhi[7:0] == ASID[0])))?1:0;
assign hit_data[1] = (vaddr_data[31:13] == VPN2[1] && (G[1] || (Entryhi[7:0] == ASID[1])))?1:0;
assign hit_data[2] = (vaddr_data[31:13] == VPN2[2] && (G[2] || (Entryhi[7:0] == ASID[2])))?1:0;
assign hit_data[3] = (vaddr_data[31:13] == VPN2[3] && (G[3] || (Entryhi[7:0] == ASID[3])))?1:0;
assign hit_data[4] = (vaddr_data[31:13] == VPN2[4] && (G[4] || (Entryhi[7:0] == ASID[4])))?1:0;
assign hit_data[5] = (vaddr_data[31:13] == VPN2[5] && (G[5] || (Entryhi[7:0] == ASID[5])))?1:0;
assign hit_data[6] = (vaddr_data[31:13] == VPN2[6] && (G[6] || (Entryhi[7:0] == ASID[6])))?1:0;
assign hit_data[7] = (vaddr_data[31:13] == VPN2[7] && (G[7] || (Entryhi[7:0] == ASID[7])))?1:0;
assign hit_data[8] = (vaddr_data[31:13] == VPN2[8] && (G[8] || (Entryhi[7:0] == ASID[8])))?1:0;
assign hit_data[9] = (vaddr_data[31:13] == VPN2[9] && (G[9] || (Entryhi[7:0] == ASID[9])))?1:0;
assign hit_data[10] = (vaddr_data[31:13] == VPN2[10] && (G[10] || (Entryhi[7:0] == ASID[10])))?1:0;
assign hit_data[11] = (vaddr_data[31:13] == VPN2[11] && (G[11] || (Entryhi[7:0] == ASID[11])))?1:0;
assign hit_data[12] = (vaddr_data[31:13] == VPN2[12] && (G[12] || (Entryhi[7:0] == ASID[12])))?1:0;
assign hit_data[13] = (vaddr_data[31:13] == VPN2[13] && (G[13] || (Entryhi[7:0] == ASID[13])))?1:0;
assign hit_data[14] = (vaddr_data[31:13] == VPN2[14] && (G[14] || (Entryhi[7:0] == ASID[14])))?1:0;
assign hit_data[15] = (vaddr_data[31:13] == VPN2[15] && (G[15] || (Entryhi[7:0] == ASID[15])))?1:0;
assign hit_data[16] = (vaddr_data[31:13] == VPN2[16] && (G[16] || (Entryhi[7:0] == ASID[16])))?1:0;
assign hit_data[17] = (vaddr_data[31:13] == VPN2[17] && (G[17] || (Entryhi[7:0] == ASID[17])))?1:0;
assign hit_data[18] = (vaddr_data[31:13] == VPN2[18] && (G[18] || (Entryhi[7:0] == ASID[18])))?1:0;
assign hit_data[19] = (vaddr_data[31:13] == VPN2[19] && (G[19] || (Entryhi[7:0] == ASID[19])))?1:0;
assign hit_data[20] = (vaddr_data[31:13] == VPN2[20] && (G[20] || (Entryhi[7:0] == ASID[20])))?1:0;
assign hit_data[21] = (vaddr_data[31:13] == VPN2[21] && (G[21] || (Entryhi[7:0] == ASID[21])))?1:0;
assign hit_data[22] = (vaddr_data[31:13] == VPN2[22] && (G[22] || (Entryhi[7:0] == ASID[22])))?1:0;
assign hit_data[23] = (vaddr_data[31:13] == VPN2[23] && (G[23] || (Entryhi[7:0] == ASID[23])))?1:0;
assign hit_data[24] = (vaddr_data[31:13] == VPN2[24] && (G[24] || (Entryhi[7:0] == ASID[24])))?1:0;
assign hit_data[25] = (vaddr_data[31:13] == VPN2[25] && (G[25] || (Entryhi[7:0] == ASID[25])))?1:0;
assign hit_data[26] = (vaddr_data[31:13] == VPN2[26] && (G[26] || (Entryhi[7:0] == ASID[26])))?1:0;
assign hit_data[27] = (vaddr_data[31:13] == VPN2[27] && (G[27] || (Entryhi[7:0] == ASID[27])))?1:0;
assign hit_data[28] = (vaddr_data[31:13] == VPN2[28] && (G[28] || (Entryhi[7:0] == ASID[28])))?1:0;
assign hit_data[29] = (vaddr_data[31:13] == VPN2[29] && (G[29] || (Entryhi[7:0] == ASID[29])))?1:0;
assign hit_data[30] = (vaddr_data[31:13] == VPN2[30] && (G[30] || (Entryhi[7:0] == ASID[30])))?1:0;
assign hit_data[31] = (vaddr_data[31:13] == VPN2[31] && (G[31] || (Entryhi[7:0] == ASID[31])))?1:0;

assign PFN_data[0] = (vaddr_data[12])?PFN1[0]:PFN0[0];
assign PFN_data[1] = (vaddr_data[12])?PFN1[1]:PFN0[1];
assign PFN_data[2] = (vaddr_data[12])?PFN1[2]:PFN0[2];
assign PFN_data[3] = (vaddr_data[12])?PFN1[3]:PFN0[3];
assign PFN_data[4] = (vaddr_data[12])?PFN1[4]:PFN0[4];
assign PFN_data[5] = (vaddr_data[12])?PFN1[5]:PFN0[5];
assign PFN_data[6] = (vaddr_data[12])?PFN1[6]:PFN0[6];
assign PFN_data[7] = (vaddr_data[12])?PFN1[7]:PFN0[7];
assign PFN_data[8] = (vaddr_data[12])?PFN1[8]:PFN0[8];
assign PFN_data[9] = (vaddr_data[12])?PFN1[9]:PFN0[9];
assign PFN_data[10] = (vaddr_data[12])?PFN1[10]:PFN0[10];
assign PFN_data[11] = (vaddr_data[12])?PFN1[11]:PFN0[11];
assign PFN_data[12] = (vaddr_data[12])?PFN1[12]:PFN0[12];
assign PFN_data[13] = (vaddr_data[12])?PFN1[13]:PFN0[13];
assign PFN_data[14] = (vaddr_data[12])?PFN1[14]:PFN0[14];
assign PFN_data[15] = (vaddr_data[12])?PFN1[15]:PFN0[15];
assign PFN_data[16] = (vaddr_data[12])?PFN1[16]:PFN0[16];
assign PFN_data[17] = (vaddr_data[12])?PFN1[17]:PFN0[17];
assign PFN_data[18] = (vaddr_data[12])?PFN1[18]:PFN0[18];
assign PFN_data[19] = (vaddr_data[12])?PFN1[19]:PFN0[19];
assign PFN_data[20] = (vaddr_data[12])?PFN1[20]:PFN0[20];
assign PFN_data[21] = (vaddr_data[12])?PFN1[21]:PFN0[21];
assign PFN_data[22] = (vaddr_data[12])?PFN1[22]:PFN0[22];
assign PFN_data[23] = (vaddr_data[12])?PFN1[23]:PFN0[23];
assign PFN_data[24] = (vaddr_data[12])?PFN1[24]:PFN0[24];
assign PFN_data[25] = (vaddr_data[12])?PFN1[25]:PFN0[25];
assign PFN_data[26] = (vaddr_data[12])?PFN1[26]:PFN0[26];
assign PFN_data[27] = (vaddr_data[12])?PFN1[27]:PFN0[27];
assign PFN_data[28] = (vaddr_data[12])?PFN1[28]:PFN0[28];
assign PFN_data[29] = (vaddr_data[12])?PFN1[29]:PFN0[29];
assign PFN_data[30] = (vaddr_data[12])?PFN1[30]:PFN0[30];
assign PFN_data[31] = (vaddr_data[12])?PFN1[31]:PFN0[31];

assign paddr_data[31:12] = 	({20{hit_data[0]}} & PFN_data[0])   | ({20{hit_data[1]}} & PFN_data[1]) |
							({20{hit_data[2]}} & PFN_data[2])   | ({20{hit_data[3]}} & PFN_data[3]) |
							({20{hit_data[4]}} & PFN_data[4])   | ({20{hit_data[5]}} & PFN_data[5]) |
							({20{hit_data[6]}} & PFN_data[6])   | ({20{hit_data[7]}} & PFN_data[7]) |
							({20{hit_data[8]}} & PFN_data[8])   | ({20{hit_data[9]}} & PFN_data[9]) |
							({20{hit_data[10]}} & PFN_data[10]) | ({20{hit_data[11]}} & PFN_data[11]) |
							({20{hit_data[12]}} & PFN_data[12]) | ({20{hit_data[13]}} & PFN_data[13]) |
							({20{hit_data[14]}} & PFN_data[14]) | ({20{hit_data[15]}} & PFN_data[15]) |
							({20{hit_data[16]}} & PFN_data[16]) | ({20{hit_data[17]}} & PFN_data[17]) |
							({20{hit_data[18]}} & PFN_data[18]) | ({20{hit_data[19]}} & PFN_data[19]) |
							({20{hit_data[20]}} & PFN_data[20]) | ({20{hit_data[21]}} & PFN_data[21]) |
							({20{hit_data[22]}} & PFN_data[22]) | ({20{hit_data[23]}} & PFN_data[23]) |
							({20{hit_data[24]}} & PFN_data[24]) | ({20{hit_data[25]}} & PFN_data[25]) |
							({20{hit_data[26]}} & PFN_data[26]) | ({20{hit_data[27]}} & PFN_data[27]) |
							({20{hit_data[28]}} & PFN_data[28]) | ({20{hit_data[29]}} & PFN_data[29]) |
							({20{hit_data[30]}} & PFN_data[30]) | ({20{hit_data[31]}} & PFN_data[31]) ;
							
assign paddr_data[11: 0] = vaddr_data[11:0];

endmodule