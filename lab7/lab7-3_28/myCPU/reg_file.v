`timescale 10ns / 1ns

module regfile(
	input         clk,
	input         rst,
	input         wen,
	input  [ 4:0] waddr,
	input  [ 4:0] raddr1,
	input  [ 4:0] raddr2,
	input  [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2
);

reg  [31:0] r [31:0];

//write
integer i;
always@(posedge clk) begin
    
    /*if(rst==1)
      for(i=0;i<32;i=i+1)
          r[i]<=0;*/
    if(wen==1 && waddr!=0)
      r[waddr]=wdata;
    end
//read out 1
assign rdata1 = (raddr1==5'b0) ? 32'b0 : r[raddr1];
//read out 2
assign rdata2 = (raddr2==5'b0) ? 32'b0 : r[raddr2];

endmodule
